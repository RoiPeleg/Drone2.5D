import time

from playground.slam.frame import Frame
from playground.slam.posegraph import PoseGraph


class BackEnd:
    def __init__(self, edge_sigma, angle_sigma):
        self.__pose_graph = PoseGraph(edge_sigma_x=edge_sigma, edge_sigma_y=edge_sigma,
                                      edge_sigma_angle=angle_sigma)

    def update_frames(self, frames: list[Frame], loop_frame: Frame):
        """
        Optimize pose graph and update frame positions with an assumption that we've detected a loop closure
        """
        start_time = time.perf_counter()
        print('Pose Graph optimization started...')
        self.__pose_graph.clear()
        vertex_index = self.__pose_graph.prior_pose_index + 1
        for frame in frames:
            ty = frame.position[0]
            tx = frame.position[1]
            rot = frame.rotation[:2, :2]
            self.__pose_graph.add_vertex(vertex_index, ty, tx, rot.T)

            edge_ty = frame.relative_icp_position[1]
            edge_tx = frame.relative_icp_position[0]
            edge_rot = frame.relative_icp_rotation[:2, :2]
            self.__pose_graph.add_factor_edge(vertex_index - 1, vertex_index, edge_ty, edge_tx, edge_rot.T)
            vertex_index += 1

        # add the loop closure constraint
        # This factor encodes the fact that we have returned to the same pose. In real
        # systems, these constraints may be identified in many ways, such as appearance-based
        # techniques with camera images.

        loop_ty = loop_frame.relative_icp_position[1]
        loop_tx = loop_frame.relative_icp_position[0]
        loop_rot = loop_frame.relative_icp_rotation[:2, :2]
        self.__pose_graph.add_factor_edge(vertex_index - 1,
                                          self.__pose_graph.prior_pose_index + 1,
                                          loop_ty, loop_tx, loop_rot.T)

        self.__pose_graph.optimize()

        vertex_index = self.__pose_graph.prior_pose_index + 1
        for frame in frames:
            tx, ty, rot = self.__pose_graph.get_pose_at(vertex_index)
            frame.position[:2] = ty, tx
            frame.rotation = rot.T
            vertex_index += 1

        end_time = time.perf_counter()
        print(f'Pose Graph optimization finished in {end_time - start_time} seconds')
