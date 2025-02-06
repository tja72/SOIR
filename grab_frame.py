import cv2
import time


def get_frames():
    capture = cv2.VideoCapture(cv2.CAP_OPENNI)
    capture.set(cv2.CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv2.CAP_OPENNI_VGA_30HZ)

    while(True):
            if not capture.grab():
                print("Unable to Grab Frames from camera")
                break
            okay, color_image = capture.retrieve(0, cv.CV_CAP_OPENNI_BGR_IMAGE)
            if not okay:
                print("Unable to retrieve Color Image from device")
                break

            cv2.imshow("rgb camera", color_image)
            name = "images/" + str(time.time()) + ".png"
            cv2.imwrite(name, color_image)
            if cv2.waitKey(10) == 27:
                break

    capture.release()

if __name__ == "__main__":
    get_frames()