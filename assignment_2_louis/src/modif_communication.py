
#----- Master

#Function
def get_apriltags_detected(self):
    rospy.wait_for_service('/get_apriltags_detected')

    try:
        coeff_service = rospy.ServiceProxy('/get_apriltags_detected', Trigger)
        response = coeff_service()

        if response.success:
            rospy.loginfo("AprilTags provided buy node_b :")
            list_apriltags = []

            # Split the apriltags information separated by ';'
            tags = response.message.split(';')

            for tag in tags:
                # Split each information separated by ','
                data   = tag.split(',')
                id_tag = int(data[0])    # ID as integer
                x      = float(data[1])  # x as float
                y      = float(data[2])  # y as float
                z      = float(data[3])  # z as float

                list_apriltags.append([id_tag, x, y, z])
                rospy.loginfo(f"- apriltag_{id_tag}:{x},{y},{z}")
                
            return list_apriltags
        
        else:
            rospy.logwarn("Failed to retrieve detected apriltags.")
            return None
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None


# Call
list_apriltags = self.get_apriltags_detected()





#----- Slave

#Call
rospy.Service('/get_apriltags_detected', Trigger, handle_apriltags_detected)


#Handler
list_apriltags = [None]*11

list_apriltags[2] = [1,2,4]
list_apriltags[4] = [-1,0,2.4]
def handle_apriltags_detected(req):

    rospy.loginfo("Providing apriltags to node_c :")

    #Format "id,x,y,z ; id,x,y,z ; ..."
    string_apriltags = ""
    for i in range(11):
        if list_apriltags[i] != None:
            string_apriltags += f"{i},{list_apriltags[i][0]},{list_apriltags[i][1]},{list_apriltags[i][2]};"
            rospy.loginfo(f"- apriltag_{i}:{list_apriltags[i][0]},{list_apriltags[i][1]},{list_apriltags[i][2]}")
    #Take out the last ";"
    if len(string_apriltags)>0:
        string_apriltags= string_apriltags[:-1]


    return TriggerResponse(success=True, message=string_apriltags)












