import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

cap = cv2.VideoCapture(0) 

def estimate_pose_from_single_marker( corners, size, camera_matrix, dist_coeffs):
        obj_pts = np.array(
            [
                [size/2, size/2, 0],
                [-size/2, size/2, 0],
                [-size/2, -size/2, 0],
                [size/2, -size/2, 0]
            ], dtype=np.float32
        )
        #print(obj_pts.shape)
        print(corners)
        ret, rvec, tvec = cv2.solvePnP(obj_pts, corners[0].reshape(
            (4, 2)).astype(float), camera_matrix, dist_coeffs)
        return ret, rvec, tvec

def draw_axis( img, camera_matrix, dist_coeff, rvec, tvec, axis_size=0.4):

    obj_pts = np.array(
        [
            [0, 0, 0],
            [axis_size, 0, 0],
            [0, axis_size, 0],
            [0, 0, axis_size]
        ], dtype=np.float32
    )
    points, _ = cv2.projectPoints(
        obj_pts, rvec, tvec, cameraMatrix=camera_matrix, distCoeffs=dist_coeff)
    points = points.astype(int).reshape((4, 2))
    cv2.line(img, points[0], points[1], (0, 0, 255), 2)
    cv2.line(img, points[0], points[2], (0, 255, 0), 2)
    cv2.line(img, points[0], points[3], (255, 0, 0), 2)


def track(matrix_coefficients, distortion_coefficients):
    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters()  # Marker detection parameters
        # lists of ids and the corners beloning to each id
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejectedCandidates = detector.detectMarkers(frame)                                                        
        #img_out = gray.copy()
        if corners:
            ret, rvec, tvec = estimate_pose_from_single_marker( corners, 1, matrix_coefficients, distortion_coefficients)
            draw_axis(frame, matrix_coefficients, None, rvec, tvec)
        cv2.imshow('input', frame)
        #cv2.imshow( 'Output', img_out)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    with open('ost.yaml', "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)

    matrix_coefficients = calib_data["camera_matrix"]["data"]
    distortion_coefficients=calib_data["distortion_coefficients"]["data"]

    track(np.array(matrix_coefficients).reshape(3,3),np.array(distortion_coefficients))
