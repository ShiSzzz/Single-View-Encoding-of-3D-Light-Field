import bpy
import os
import mathutils
import math
from math import atan, degrees, radians, tan, sqrt, floor, cos, sin
import numpy as np


def cameras_path(num_cameras,hemi_radius):
    # 删除现有相机（可选）
    for obj in bpy.data.objects:
        if obj.type == 'CAMERA':
            bpy.data.objects.remove(obj)

    golden_angle = math.pi * (3 - math.sqrt(5))  # 约137.508度
    camera_positions = []
    camera_objects = []

    for i in range(num_cameras):
        # 计算z坐标（限制在半球范围）
        y_progress = 1 - 0.7 * i / (num_cameras - 1) if num_cameras > 1 else 0.5
        y = y_progress  # [0, 1] 从顶部到底部

        # 计算球面坐标
        theta = golden_angle * i
        radius_xy = sqrt(1 - y * y)  # 保持点在单位球面

        # 转换为笛卡尔坐标并缩放
        z = radius_xy * cos(theta) * hemi_radius
        x = radius_xy * sin(theta) * hemi_radius
        y = y * hemi_radius

        camera_positions.append((x, y, z))

    # 创建相机并设置朝向
    for idx, pos in enumerate(camera_positions):
        # 创建相机
        cam_data = bpy.data.cameras.new(f"HemiCam_{idx + 1:03d}")
        cam_obj = bpy.data.objects.new(f"HemiCam_{idx + 1:03d}", cam_data)
        bpy.context.collection.objects.link(cam_obj)

        # 设置位置
        cam_obj.location = pos

        # 计算朝向原点的方向向量
        direction = mathutils.Vector(pos).normalized()  # 指向原点

        # 使用四元数计算旋转
        track_axis = 'Z'  # 相机默认朝向-Z轴
        up_axis = 'Y'  # 上方向轴

        # 计算四元数旋转
        rot_quat = direction.to_track_quat(track_axis[-1], up_axis)

        # 应用旋转
        cam_obj.rotation_mode = 'QUATERNION'
        cam_obj.rotation_quaternion = rot_quat
        cam_obj.rotation_mode = 'XYZ'  # 切换回欧拉角（可选）
        camera_objects.append(cam_obj)
    # 可选：创建一个中心参考物体
    # bpy.ops.object.empty_add(type='SPHERE', location=(0, 0, 0))
    # bpy.context.object.name = "Hemisphere_Center"
    return camera_objects
    print(f"成功创建 {num_cameras} 个半球分布相机")


#创建相机
camera_objects = cameras_path(250,2)

# 获取当前场景
scene = bpy.context.scene

# 获取当前相机
#camera = scene.camera



## 渲染的起始帧和结束帧
#start_frame = scene.frame_start
#end_frame = scene.frame_end

# 自定义工作空间路径
workspace_path = "F:/tools/dataset"  # 例如: "C:/Users/YourUser/Documents/BlenderWorkspace"
obj_folder = os.path.join(workspace_path, "test/5")  # 存放.obj模型的文件夹

image_folder =os.path.join(workspace_path, "dataset_20250207/cars_test")

# 获取图像分辨率
resolution_x = scene.render.resolution_x
resolution_y = scene.render.resolution_y

# 获取相机的内参矩阵
camera_data = camera_objects[0].data
sensor_width = camera_data.sensor_width
sensor_height = camera_data.sensor_height

# 计算相机的FOV（视场角）
fov_x = camera_data.angle_x  # 水平FOV
fov_y = camera_data.angle_y  # 垂直FOV

# 获取焦距 fx 和 fy
fx = (resolution_x / 2) / tan(fov_x / 2)
fy = (resolution_y / 2) / tan(fov_y / 2)


count = 400

# 遍历obj_folder下的所有子文件夹
for root, dirs, files in os.walk(obj_folder):
    for file in files:
        if file.endswith('.obj'):
            # 获取当前.obj文件的路径
            obj_file_path = os.path.join(root, file)

            rgb_folder = os.path.join(image_folder, f"{count:06d}/RGB")
            pose_folder = os.path.join(image_folder, f"{count:06d}/pose")

            # 创建RGB和pose文件夹（如果不存在的话）
            os.makedirs(rgb_folder, exist_ok=True)
            os.makedirs(pose_folder, exist_ok=True)

            camera_params_txt_path = os.path.join(image_folder, f"{count:06d}/intrinsics.txt")
            # 输出相机参数和分辨率到一个TXT文件
            with open(camera_params_txt_path, "w") as param_file:
                param_file.write(f"{fov_x}\n")
                param_file.write(f"{fov_y}\n")
                param_file.write(f"{fx}\n")
                param_file.write(f"{fy}\n")
                param_file.write(f"{resolution_x}x{resolution_y}\n")

            # 加载.obj模型
            bpy.ops.import_scene.obj(filepath=obj_file_path)
            print(f"Loaded {file} from {root}")

            # 启动逐帧渲染
            for frame, camera in enumerate(camera_objects):
                # 设置当前帧
                bpy.context.scene.camera = camera
                scene.frame_set(frame)

                # 渲染当前帧
                output_image_path = os.path.join(rgb_folder,  f"{frame:06d}.png")
                bpy.context.scene.render.filepath = output_image_path
                bpy.ops.render.render(write_still=True)

                # 获取相机位姿矩阵
                camera_matrix = camera.matrix_world
                
                c2w_matrix = np.array(camera_matrix)
                
                rotation_matrix = c2w_matrix[:3, :3]
                
                # 反转旋转矩阵（翻转方向：镜像 X、Y、Z 轴）
                rotation_matrix_reversed = np.dot(rotation_matrix, np.diag([-1, -1, -1]))
    


                new_c2w_matrix = c2w_matrix.copy()
                new_c2w_matrix[:3, :3] = rotation_matrix_reversed  # 替换旋转矩阵
                
#                blender_adjust = np.array([
#                    [1,  0,  0],
#                    [0, -1, 0],
#                    [0,  0, 1]
#                ])
#                rotation_matrix = rotation_matrix @ blender_adjust
#                c2w_matrix[:3, :3] = rotation_matrix

                # 坐标系变换
#                c2w_matrix = np.array(camera_matrix)

#                M_axis = np.array([
#                    [1, 0, 0, 0],
#                    [0, 0, -1, 0],
#                    [0, 1, 0, 0],
#                    [0, 0, 0, 1]
#                ])

#                c2w_colmap = M_axis @ c2w_matrix


                # 保存相机位姿矩阵到TXT文件
                output_pose_path = os.path.join(pose_folder, f"{frame:06d}.txt")
                with open(output_pose_path, "w") as pose_file:
                    # 将4x4矩阵逐行写入TXT
                    for row in new_c2w_matrix:
                        pose_file.write(f"{row[0]} {row[1]} {row[2]} {row[3]} ")

                print(f"Frame {frame} for {file}: Rendered and saved pose.")

            # 删除当前导入的.obj模型
            imported_objects = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']
            for obj in imported_objects:
                obj.select_set(True)
            bpy.ops.object.delete()  # 删除选中的模型
            
            count = count + 1           
            #print(f"Deleted model {file}, ready to load next one.")
    if count ==  500:
        break


