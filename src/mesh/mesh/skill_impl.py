from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, GoalResponse
from mesh_skill_msgs.action import Mesh
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import open3d as o3d
import numpy as np

DATA_DIR = "/opt/scans"
MOLT = 1


class MeshSkillImpl(Node):

    def __init__(self) -> None:

        super().__init__('skill_mesh')

        self.declare_parameter(
            'parameter_name', 'parameter_value',
            ParameterDescriptor(description='A parameter for the skill')
        )

        self.get_logger().info("Initialising...")
        self._timer = None
        self._diag_pub = None
        self._diag_timer = None
        self._nb_requests = 0

        self.get_logger().info('Skill mesh started, but not yet configured.')

    def calculate_normals(self, pcd):
        # Check if points exist
        if len(pcd.points) == 0:
            self.get_logger().error("Error: Point cloud is empty.")
        else:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=8.0 * MOLT, max_nn=190))

            pcd.orient_normals_consistent_tangent_plane(20)

            # Check if normals were estimated
            if len(pcd.normals) == 0:
                self.get_logger().error("Error: Normals estimation failed.")
        return pcd

    def remove_less_dense_regions(self, rec_mesh, densities):
        self.get_logger().info(f"Cleaning mesh")
        threshold = np.quantile(densities, 0.01)
        vertices_to_remove = densities < threshold
        rec_mesh.remove_vertices_by_mask(vertices_to_remove)
        return rec_mesh

    def double_sided_mesh(self, mesh):
        # Extract original triangles
        triangles = np.asarray(mesh.triangles)

        # Create reversed triangles
        reversed_triangles = triangles[:, [0, 2, 1]]

        # Combine original and reversed triangles
        double_triangles = np.vstack([triangles, reversed_triangles])

        # Create a new mesh with double-sided faces
        double_mesh = o3d.geometry.TriangleMesh()
        double_mesh.vertices = mesh.vertices
        double_mesh.triangles = o3d.utility.Vector3iVector(double_triangles)

        # Set colors
        if mesh.has_vertex_colors():
            vertex_colors = np.asarray(mesh.vertex_colors)
            double_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

        return double_mesh

    def reconstruct_mesh(self, scan_id=None, depth=6, target_triangles=50000):
        """
        Mesh Reconstruction from pcl scan_id
        """

        pcd_file = f"{DATA_DIR}/{scan_id}/pcl.ply"
        pcd = o3d.io.read_point_cloud(pcd_file)

        self.get_logger().info("Reconstructing mesh")

        # Calculate Normals
        pcd = self.calculate_normals(pcd)

        # Remove outliers
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        pcd = pcd.select_by_index(ind)

        # Poisson reconstruction
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            rec_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

        # Remove less dense regions
        rec_mesh = self.remove_less_dense_regions(rec_mesh, densities)

        # Decimate mesh
        rec_mesh = rec_mesh.simplify_quadric_decimation(target_triangles)

        self.get_logger().info(f"Making double-sided mesh")

        # Double side mesh
        double_mesh = self.double_sided_mesh(rec_mesh)
        double_mesh.compute_vertex_normals()

        # Write ply
        pcd_out_file = f'{DATA_DIR}/{scan_id}/mesh_{scan_id}.ply'
        pcd_out_file_d = f'{DATA_DIR}/{scan_id}/mesh_{scan_id}_double.ply'

        self.get_logger().info(f"Writing mesh: {pcd_out_file}")
        o3d.io.write_triangle_mesh(pcd_out_file, rec_mesh, write_ascii=False, compressed=False)

        self.get_logger().info(f"Writing mesh: {pcd_out_file_d}")
        o3d.io.write_triangle_mesh(pcd_out_file_d, double_mesh, write_ascii=False, compressed=False)
        return pcd_out_file_d

    def run_skill(self, scan_id=None):

        self.get_logger().info("...running the skill")

        # Tuned Parameters
        result = -1
        target_triangles = 300000
        depth = 11

        try:
            out_file = self.reconstruct_mesh(scan_id=scan_id, depth=depth, target_triangles=target_triangles)
            result = scan_id
        except Exception as e:
            self.get_logger().error(f"{e}")

        return result

    def on_request_goal(self, goal_handle):
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("Skill is not active yet, rejecting goal")
            return GoalResponse.REJECT

        self.get_logger().info("Accepted a new goal")
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        self.get_logger().info(f"Executing the skill with data: {goal_handle.request.scan_id}")

        # perform request here
        res = self.run_skill(goal_handle.request.scan_id)

        self.get_logger().info("Goal executed successfully")
        goal_handle.succeed()

        return Mesh.Result(result=res)

    #################################

    def on_configure(self, state: State) -> TransitionCallbackReturn:

        self.get_logger().info("Skill mesh start configuration")
        self._nb_requests = 0

        self.skill_server = ActionServer(self,
                                         Mesh,
                                         "mesh_reference",
                                         goal_callback=self.on_request_goal,
                                         execute_callback=self.on_request_exec)

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self._diag_timer = self.create_timer(1., self.publish_diagnostics)

        self.get_logger().info("Skill mesh is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:

        timer_period = 1  # in sec
        self._timer = self.create_timer(timer_period, self.run)

        self.get_logger().info("Skill mesh is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Stopping skill...")
        self.destroy_timer(self._timer)

        self.get_logger().info("Skill mesh is stopped (inactive)")
        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:

        self.get_logger().info('Shutting down mesh skill.')

        self.skill_server.destroy()

        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.get_logger().info("Skill mesh finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/skill/mesh",
            message="skill mesh is running",
            values=[
                KeyValue(key="Module name", value="mesh"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="# invokations", value=str(self._nb_requests)),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)

    def run(self) -> None:

        self.get_logger().info("skill mesh: performing background task...")
