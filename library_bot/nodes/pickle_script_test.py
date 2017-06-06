import pickle
from database_reader import BookInfo
from geometry_msgs.msg import Pose
#import main_runner

IN_SIM = False
DEMO_SAVE = False

#Simulation stuff
books = None
bookPath = None

#if main_runner.IN_SIM:
if IN_SIM:
    # At bookshelf
    bookshelfPose = Pose()
    bookshelfPose.position.x = 2.56
    bookshelfPose.position.y = 3.22
    bookshelfPose.position.z = 0.0
    bookshelfPose.orientation.x = 0.0
    bookshelfPose.orientation.y = 0.0
    bookshelfPose.orientation.z = 0.016551
    bookshelfPose.orientation.w = 1.0

    returnPose = Pose()
    returnPose.position.x = 0.3548
    returnPose.position.y = 0.6489
    returnPose.position.z = 0.0
    returnPose.orientation.x = 0.0
    returnPose.orientation.y = 0.0
    returnPose.orientation.z = 0.14559
    returnPose.orientation.w = .989

    book1 = BookInfo()
    book1.pose = bookshelfPose
    book1.torso_height = 0.4
    book1.head_pan = -0.15
    book1.head_tilt = 0.3
    book1.book_name = "Theoretical Neuroscience"
    book1.fiducial_number = 4

    #At bookshelf
    book2 = BookInfo()
    book2.pose = bookshelfPose
    book2.torso_height = 0.4
    book2.head_pan = -0.15
    book2.head_tilt = 0.3
    book2.book_name = "Introduction to Robotics"
    book2.fiducial_number = 13

    book3 = BookInfo()
    book3.pose = bookshelfPose
    book3.torso_height = 0.4
    book3.head_pan = -0.15
    book3.head_tilt = 0.3
    book3.book_name = "Praise Justin"
    book3.fiducial_number = 13

    home1 = BookInfo()
    home1.pose = returnPose
    home1.torso_height = 0.0
    home1.head_pan = -0.0
    home1.head_tilt = 0.0
    home1.book_name = "Theoretical Neuroscience"
    home1.fiducial_number = 4

    delivery1 = BookInfo()
    delivery1.pose = returnPose
    delivery1.torso_height = 0.0
    delivery1.head_pan = -0.0
    delivery1.head_tilt = 0.0
    delivery1.book_name = "Theoretical Neuroscience"
    delivery1.fiducial_number = 4

    books = {1: book1, 2: book2, 3: book3, -1: home1, -2: delivery1}
    bookPath = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_sim.p"
elif DEMO_SAVE:

    #TODO: Fill out for demo

    # Home
    homePose = Pose()
    homePose.position.x = 0.0
    homePose.position.y = 0.0
    homePose.position.z = 0.0
    homePose.orientation.x = 0.0
    homePose.orientation.y = 0.0
    homePose.orientation.z = 0.0
    homePose.orientation.w = 0.0

    # Delivery
    # From the left front of the table, measure 28.5cm towards the middle.
    # From there, measure from that horizontal distance @ the ground, 
    # 37.5cm from there, perpendicular to the table to the front of the robot chassis
    deliveryPose = Pose()
    deliveryPose.position.x = 0.0
    deliveryPose.position.y = 0.0
    deliveryPose.position.z = 0.0
    deliveryPose.orientation.x = 0.0
    deliveryPose.orientation.y = 0.0
    deliveryPose2orientation.z = 0.0
    deliveryPose.orientation.w = 0.0

    # At bookshelf:
    # 63.5cm from the bottom shelf level to the front of the robot chassis
    bookshelfPose = Pose()
    bookshelfPose.position.x = 0.0
    bookshelfPose.position.y = 0.0
    bookshelfPose.position.z = 0.0
    bookshelfPose.orientation.x = 0.0
    bookshelfPose.orientation.y = 0.0
    bookshelfPose.orientation.z = 0.0
    bookshelfPose.orientation.w = 0.0

    book1 = BookInfo()
    book1.pose = bookshelfPose
    book1.torso_height = 0.30
    book1.head_pan = 0.0
    book1.head_tilt = 0.6
    book1.book_name = "Markov Random Fields"
    book1.fiducial_number = 13

    book2 = BookInfo()
    book2.pose = bookshelfPose
    book2.torso_height = 0.30
    book2.head_pan = 0.0
    book2.head_tilt = 0.6
    book2.book_name = "Engineering and Chemical Thermodynamics"
    book2.fiducial_number = 4

    book3 = BookInfo()
    book3.pose = bookshelfPose
    book3.torso_height = 0.30
    book3.head_pan = 0.0
    book3.head_tilt = 0.6
    book3.book_name = "Electronic Music"
    book3.fiducial_number = 5

    book4 = BookInfo()
    book4.pose = bookshelfPose
    book4.torso_height = 0.30
    book4.head_pan = 0.0
    book4.head_tilt = 0.6
    book4.book_name = "TCP/IP (Yoga)"
    book4.fiducial_number = 3

    home1 = BookInfo()
    home1.pose = homePose
    home1.torso_height = 0.0
    home1.head_pan = 0.0
    home1.head_tilt = 0.0
    home1.book_name = "Praise Justin"
    home1.fiducial_number = 0

    delivery1 = BookInfo()
    delivery1.pose = deliveryPose
    delivery1.torso_height = 0.0
    delivery1.head_pan = 0.0
    delivery1.head_tilt = 0.0
    delivery1.book_name = "Praise Justin"
    delivery1.fiducial_number = 0

    books = {1: book1, 2: book2, 3: book3, 4: book4, -1: home1, -2: delivery1}
    bookPath = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_DEMO.p"

else:
    homePose = Pose()
    homePose.position.x = -0.905816028694
    homePose.position.y = -15.1832829877
    homePose.position.z = 0.0
    homePose.orientation.x = 0.0
    homePose.orientation.y = 0.0
    homePose.orientation.z = 0.991462778289
    homePose.orientation.w = 0.130390027485

    # deliveryPose1 = Pose()
    # deliveryPose1.position.x = -2.71640902391
    # deliveryPose1.position.y = -14.6933606141
    # deliveryPose1.position.z = 0.0
    # deliveryPose1.orientation.x = 0.0
    # deliveryPose1.orientation.y = 0.0
    # deliveryPose1.orientation.z = 0.00472233879895
    # deliveryPose1.orientation.w = 0.999988849696

    deliveryPose = Pose()
    deliveryPose.position.x = -4.31286643672
    deliveryPose.position.y = -14.6842134712
    deliveryPose.position.z = 0.0
    deliveryPose.orientation.x = 0.0
    deliveryPose.orientation.y = 0.0
    deliveryPose.orientation.z = 0.980614521179
    deliveryPose.orientation.w = 0.1959468317

    # At bookshelf
    bookshelfPose = Pose()
    bookshelfPose.position.x = -3.66569411557
    bookshelfPose.position.y = -14.5338443433
    bookshelfPose.position.z = 0.0
    bookshelfPose.orientation.x = 0.0
    bookshelfPose.orientation.y = 0.0
    bookshelfPose.orientation.z = 0.65705315908
    bookshelfPose.orientation.w = 0.753844245281

    # Old poses we don't really have a use for
 
    # bookshelfPose2 = Pose()
    # bookshelfPose2.position.x = -3.55608035699
    # bookshelfPose2.position.y = -14.5941557214
    # bookshelfPose2.position.z = 0.0
    # bookshelfPose2.orientation.x = 0.0
    # bookshelfPose2.orientation.y = 0.0
    # bookshelfPose2.orientation.z = 0.611408792497
    # bookshelfPose2.orientation.w = 0.791314911054

    # bookshelfPose3 = Pose()
    # bookshelfPose3.position.x = -3.60172798139
    # bookshelfPose3.position.y = -14.7564795155
    # bookshelfPose3.position.z = 0.0
    # bookshelfPose3.orientation.x = 0.0
    # bookshelfPose3.orientation.y = 0.0
    # bookshelfPose3.orientation.z = 0.69354384818
    # bookshelfPose3.orientation.w = 0.720414415911

    # bookshelfPose4 = Pose()
    # bookshelfPose4.position.x = -3.60172798139
    # bookshelfPose4.position.y = -14.7564795155
    # bookshelfPose4.position.z = 0.0
    # bookshelfPose4.orientation.x = 0.0
    # bookshelfPose4.orientation.y = 0.0
    # bookshelfPose4.orientation.z = 0.69354384818
    # bookshelfPose4.orientation.w = 0.720414415911


    book1 = BookInfo()
    book1.pose = bookshelfPose
    book1.torso_height = 0.30
    book1.head_pan = 0.0
    book1.head_tilt = 0.6
    book1.book_name = "Theoretical Neuroscience"
    book1.fiducial_number = 13

    book2 = BookInfo()
    book2.pose = bookshelfPose
    book2.torso_height = 0.30
    book2.head_pan = 0.0
    book2.head_tilt = 0.6
    book2.book_name = "Introduction to Robotics"
    book2.fiducial_number = 4

    book3 = BookInfo()
    book3.pose = bookshelfPose
    book3.torso_height = 0.30
    book3.head_pan = 0.0
    book3.head_tilt = 0.55
    book3.book_name = "Praise Justin"
    book3.fiducial_number = 5

    book4 = BookInfo()
    book4.pose = bookshelfPose
    book4.torso_height = 0.30
    book4.head_pan = 0.0
    book4.head_tilt = 0.6
    book4.book_name = "Praise Justin"
    book4.fiducial_number = 3

    home1 = BookInfo()
    home1.pose = homePose
    home1.torso_height = 0.0
    home1.head_pan = 0.0
    home1.head_tilt = 0.0
    home1.book_name = "Praise Justin"
    home1.fiducial_number = 0

    delivery1 = BookInfo()
    delivery1.pose = deliveryPose
    delivery1.torso_height = 0.0
    delivery1.head_pan = 0.0
    delivery1.head_tilt = 0.0
    delivery1.book_name = "Praise Justin"
    delivery1.fiducial_number = 0

    books = {1: book1, 2: book2, 3: book3, 4: book4, -1: home1, -2: delivery1}
    bookPath = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_real_robot.p"

print books
pickle.dump(books, open(bookPath, "wb"))

# # Test to make sure reading from pickle file works

# pickle_test = pickle.load(open("./book_database.p", "rb"))
# print pickle_test[1].torso_height

# The torso_height, head_pan, and head_tilt assume the point cloud "topShelfNoHand.bag"