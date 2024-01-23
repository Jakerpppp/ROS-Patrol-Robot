from states.yolo_state import YOLOState
from geometry_msgs.msg import Point

'''Child Classes to Process YOLO Detections for each Room'''
class YOLOStateA(YOLOState):
    def __init__(self):
        super().__init__()
        self.cat_found = False
        self.dog_found = False


class YOLOStateB(YOLOState):
    def __init__(self):
        super().__init__()
        self.cat_found = False
        self.dog_found = False


class YOLOStateD(YOLOState):
    def __init__(self):
        super().__init__()
        self.person_found = False

    def process_detections(self, ud, detections):
        for detection in detections:
            if detection.class_name == 'person':
                self.person_found = True

        if self.person_found:
            rule_broken = 1
            self.get_current_coordinates()
            coords = Point(self.x, self.y, self.z)
            ud.feedback.append([coords, rule_broken])
            ud.result[0] += 1
            self.tts_pub.publish("Could all people please leave the kitchen immediately")
            self.person_found = False

    def reset_room(self):
        self.person_found = False