import numpy as np
from main import calib
import cv2
import yaml

#with open('valeo_cam.yaml', "r") as file_handle:
with open('calibrationdata_office_new_basler/ost.yaml', "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
        matrix_coefficients =    calib_data["camera_matrix"]["data"]
        distortion_coefficients = calib_data["distortion_coefficients"]["data"]
        matrix_coefficients = np.array(matrix_coefficients).reshape(3,3)
        distortion_coefficients = np.array(distortion_coefficients)

def axes_change(obj_points):
    index = [] 
    
    
    for i in range(0,len(obj_points),5):
        index.append(i + 4)
    index = np.array(index)
    centroids = obj_points[index,:]
    obj_points = np.delete(obj_points,index,0)
    #print(obj_points)
    obj_points_updated = []
    for points in obj_points:
        new_axes = np.zeros(3)
        new_axes[0]  = - points[1]  
        new_axes[1]  = - points[2]
        new_axes[2]  =   points[0]
        obj_points_updated.append(new_axes)
        
    obj_points_updated = np.array(obj_points_updated)
    #print(obj_points_updated)
    return obj_points_updated, centroids


def remove_centroid_img(img):
    index = [] 
    
    
    for i in range(0,len(img),5):
        index.append(i + 4)
    index = np.array(index)
    centroids = img[index,:]
    img = np.delete(img,index,0)
    #print(img)
    return img, centroids


def calculate_ext_param_comulative(objectPoints,imagePoints,mat_intrinsic,dist_coeffs):
    objectPoints =np.array(objectPoints)
    imagePoints = np.array(imagePoints)
    #print(objectPoints.shape)
    #print(imagePoints.shape)
    objectPoints = objectPoints.reshape(-1,objectPoints.shape[-1])
    imagePoints = imagePoints.reshape(-1,imagePoints.shape[-1])
    print(objectPoints.shape)
    print(imagePoints.shape)
    assert(objectPoints.shape[0]  == imagePoints.shape[0] )
    success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,dist_coeffs,iterationsCount=1000000,reprojectionError=1.0,confidence=0.99)
    #success, rvec, tvec,repro = cv2.solvePnPGeneric(objectPoints  ,imagePoints ,mat_intrinsic,dist_coeffs, 	flags = cv2.SOLVEPNP_P3P)
    #rvec_refine, tvec_refine = cv2.solvePnPRefineVVS(objectPoints[inliers],imagePoints[inliers], mat_intrinsic, dist_coeffs,rvec,tvec)
    print('tvec',tvec)
    print('rvec',rvec)
    print('inliers',inliers)
    points2D_reproj = project_lidar_data(objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs)
    points2D_reproj = np.squeeze(points2D_reproj)
    print('sucess', success)
    #print(points2D_reproj.shape)
    #print(imagePoints.shape)
    
    if(points2D_reproj.shape == imagePoints.shape):
        error = (points2D_reproj - imagePoints)[inliers]  # Compute error only over inliers.
        print(error.shape)
        error = np.squeeze(error)
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        print('comulative rmse',rmse)
    return rvec, tvec

def project_lidar_data(objectPoints, rvec, tvec, mat_intrinsic, dist_coeffs):
    print('object points data',objectPoints.shape)
    print('object points data',type(objectPoints))
    points_2D, _ = cv2.projectPoints(objectPoints,rvec,tvec,mat_intrinsic , dist_coeffs)
    return points_2D


def transformation_matrix(rvec, tvec):
    R_lidar_to_camera, _ = cv2.Rodrigues(rvec)

    # Compose transformation matrix T (LIDAR to camera)
    T_lidar_to_camera = np.eye(4)
    T_lidar_to_camera[:3, :3] = R_lidar_to_camera
    
    T_lidar_to_camera[:3, 3] = tvec.flatten()
    #print('T_lidar_to_camera',T_lidar_to_camera)
    # Invert T to get camera to LIDAR transformation
    T_camera_to_lidar = np.linalg.inv(T_lidar_to_camera)

    # Projection matrix from camera to LIDAR
    # Projection matrix from camera to LIDAR
    #P_camera_to_lidar = np.vstack((T_camera_to_lidar[:3], [0, 0, 0, 1]))

    print("Projection Matrix (LIDAR to Camera):\n", T_lidar_to_camera)
    print("Projection Matrix (Camera to LIDAR):\n", T_camera_to_lidar)
    print("T distance", np.sqrt(tvec[0]**2 +tvec[1]**2 + tvec[2]**2) )

def re_proj_error_comulative(points2D_reproj,points2D ):
    points2D_reproj = np.squeeze(points2D_reproj)
    print('repro')
    #print(int(points2D_reproj))
    #print(int(points2D))
    
    if(points2D_reproj.shape == points2D.shape):
        error = (points2D_reproj - points2D)#[inliers]  # Compute error only over inliers.
    
        #error = error.reshape(5,2)
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        print('comulative rmse',rmse)
    else:
        print('fix rmase, it is not working')
    
    return 0

img = np.loadtxt('img_points.txt', delimiter=',')#[:10,:] 
obj = np.loadtxt('lidar_points.txt', delimiter=',')#[:10,:]   

#img0 = np.array([[430.24606 ,389.49933],[499.0724,  283.84576],[602.85175 ,353.7939],[533.954  , 457.62646] ] ,dtype=np.float64 )
#obj0 = np.array([[2.5424461364746094,  0.6509208083152771 , -0.11368778347969055],[2.5529308319091797, 0.4567108750343323 , 0.17534801363945007],[ 2.5432772636413574 , 0.1519027203321457, 0.01405498385429383 ], [2.543548107147217, 0.3417147994041443, -0.2980373203754425]] ,dtype=np.float64 ) 
#img1 = np.array([[454.53903 ,407.74576],[511.0563 , 311.81967],[605.7542 , 367.0505],[549.90356 ,463.4666] ] ,dtype=np.float64 )
#obj1 = np.array([[2.2164275646209717,  0.6399431824684143 ,-0.03561326861381531],[2.2150022983551025, 0.44038957357406616 ,0.23735907673835754],[ 2.2959046363830566 , 0.13862542808055878, 0.04398968815803528 ], [2.2723119258880615, 0.3476335406303406, -0.26453468203544617]] ,dtype=np.float64 ) 
#img = np.concatenate((img0,img1),axis=0)
#obj = np.concatenate((obj0,obj1),axis=0)
rvec ,tvec = calculate_ext_param_comulative(obj[:,:]     ,img[:,:]     ,matrix_coefficients,distortion_coefficients)
transformation_matrix(rvec, tvec)



#rmse = re_proj_error_comulative(points2D_reproj,img)
#print('rmse',rmse)