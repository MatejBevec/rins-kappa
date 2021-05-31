

class FaceFilter():

    self.faces = []

    def __init__(self):
        rospy.Subscriber('confirmed_faces', MarkerArray, on_confirmed_detection)

    def on_confirmed_detection(self, data):
        pos = data.pose.position
        print(f"Confirmed FACE detection at ({pos.x},{pos.y})")
        new_face = {
            "position": pos,
            "mask": False
        }
        self.faces.append(new_face)

    def get_final_detections(self):
        return self.faces

if __name__ == "__main__":
    rospy.init_node("face_filter", anonymous=False)
    ff = FaceFilter()
    rospy.spin()
