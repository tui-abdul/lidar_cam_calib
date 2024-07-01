import cv2
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points

def overlay(image_msg,lidar_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        undistorted_image = cv2.undistort(image, self.matrix_coefficients,self.distortion_coefficients)
        points_gen =  read_points(lidar_msg, field_names=('x','y','z','intensity'), skip_nans=True)
        points_array = np.fromiter(points_gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity',np.float32)])
        points_array = np.asarray(points_array)
        pc_as_numpy_ring = self.fix_data_type(points_array)
        lidar_points = pc_as_numpy_ring[:,:3]
        lidar_points = np.array(lidar_points,dtype=np.float32)
        intensities = pc_as_numpy_ring[:,3]  
        lidar_points = self.trasnformation(lidar_points)[:,:3] 

        # Project the points
        
        projected_points, _ = cv2.projectPoints(lidar_points, self.rvec, self.tvec, self.matrix_coefficients, self.distortion_coefficients)
        # Convert points to integer for visualization
        projected_points = projected_points.squeeze().astype(int)

        # Image dimensions
        height, width = image.shape[:2]

        # Filter points within the image boundaries
        valid_points = []
        valid_intensities = []
        for i, point in enumerate(projected_points):
            x, y = point
            if 0 <= x < width and 0 <= y < height:
                valid_points.append((x, y))
                valid_intensities.append(intensities[i])
        
        # Convert to numpy array
        #valid_points = np.array(valid_points)
        valid_points = np.array(valid_points, dtype=np.float32).reshape(-1, 1, 2)
        valid_intensities = np.array(valid_intensities)
        # Undistort valid points
        undistorted_points = cv2.undistortPoints(valid_points, self.matrix_coefficients, self.distortion_coefficients, P=self.matrix_coefficients)
        undistorted_points = undistorted_points.reshape(-1, 2).astype(int)

        # Normalize intensity values to [0, 1] range
        normalized_intensities = (valid_intensities - valid_intensities.min()) / (valid_intensities.max() - valid_intensities.min())

        # Apply colormap
        colormap = plt.get_cmap('jet')
        colors = (colormap(normalized_intensities)[:, :3] * 255).astype(np.uint8)

        # Visualize valid LiDAR points on the image
        for point, color in zip(undistorted_points, colors):
            x, y = point
            cv2.circle(undistorted_image, (x, y), radius=3, color=(int(color[0]), int(color[1]), int(color[2])), thickness=-1)
            #cv2.circle(image, (x, y), radius=3,color=[0,0,255] , thickness=-1)
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL) 
    
        cv2.imshow('Image',undistorted_image)
        #cv2.imshow('Image',image)

        while True:
            key = cv2.waitKey(0)
            if key == 27:  # 27 is the ASCII code for the Escape key
                cv2.destroyAllWindows()
                break
        pass