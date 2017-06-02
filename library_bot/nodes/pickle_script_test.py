import pickle
from database_reader import BookInfo
from geometry_msgs.msg import Pose
#import main_runner

IN_SIM = False

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
else:
    # # Homes
    # home1 = Pose()
    # home1.position.x = -6.41489100819
    # home1.position.y = -15.1735196104
    # home1.position.z = 0.0
    # home1.orientation.x = 0.0
    # home1.orientation.y = 0.0
    # home1.orientation.z = 0.149974035061
    # home1.orientation.w = 0.988689935626

    # # (by the table (in front of the table facing the door))
    # home2 = Pose()
    # home2.position.x = -5.30197472907
    # home2.position.y = -14.0397116951
    # home2.position.z = 0.0
    # home2.orientation.x = 0.0
    # home2.orientation.y = 0.0
    # home2.orientation.z = 0.0757832579966
    # home2.orientation.w = 0.997124314119

    # # (delivery1, facing the wall towards our side)
    # home3 = Pose()
    # home3.position.x = -6.27061130366
    # home3.position.y = -15.171756664
    # home3.position.z = 0.0
    # home3.orientation.x = 0.0
    # home3.orientation.y = 0.0
    # home3.orientation.z = -0.787140411999
    # home3.orientation.w = 0.616773841694

    # #  (similar to home1, further from the wall)
    # home4 = Pose()
    # home4.position.x = -6.24471121968
    # home4.position.y = -15.1730662323
    # home4.position.z = 0.0
    # home4.orientation.x = 0.0
    # home4.orientation.y = 0.0
    # home4.orientation.z = -0.0369989396493
    # home4.orientation.w = 0.999315304829


    # # At bookshelf
    # bookshelfPose1 = Pose()
    # bookshelfPose1.position.x = -3.56270775501
    # bookshelfPose1.position.y = -14.4083569282
    # bookshelfPose1.position.z = 0.0
    # bookshelfPose1.orientation.x = 0.0
    # bookshelfPose1.orientation.y = 0.0
    # bookshelfPose1.orientation.z = 0.661069986852
    # bookshelfPose1.orientation.w = 0.750324244899

    # # Medium 
    # bookshelfPose2 = Pose()
    # bookshelfPose2.position.x = -3.57330127269
    # bookshelfPose2.position.y = -14.2583226251
    # bookshelfPose2.position.z = 0.0
    # bookshelfPose2.orientation.x = 0.0
    # bookshelfPose2.orientation.y = 0.0
    # bookshelfPose2.orientation.z = 0.676448113503
    # bookshelfPose2.orientation.w = 0.73649029168

    # # Closest
    # bookshelfPose3 = Pose()
    # bookshelfPose3.position.x = -3.56819814205
    # bookshelfPose3.position.y = -14.2622281449
    # bookshelfPose3.position.z = 0.0
    # bookshelfPose3.orientation.x = 0.0
    # bookshelfPose3.orientation.y = 0.0
    # bookshelfPose3.orientation.z = 0.688425807341
    # bookshelfPose3.orientation.w = 0.725306768055

    # # Furthest back
    # bookshelfPose4 = Pose()
    # bookshelfPose4.position.x = -3.60625646394
    # bookshelfPose4.position.y = -14.4696206429
    # bookshelfPose4.position.z = 0.0
    # bookshelfPose4.orientation.x = 0.0
    # bookshelfPose4.orientation.y = 0.0
    # bookshelfPose4.orientation.z = 0.686848774251
    # bookshelfPose4.orientation.w = 0.726800358633

    # New poses

    homePose1 = Pose()
    homePose1.position.x = -0.905816028694
    homePose1.position.y = -15.1832829877
    homePose1.position.z = 0.0
    homePose1.orientation.x = 0.0
    homePose1.orientation.y = 0.0
    homePose1.orientation.z = 0.991462778289
    homePose1.orientation.w = 0.130390027485

    deliveryPose1 = Pose()
    deliveryPose1.position.x = -1.20871210123
    deliveryPose1.position.y = -15.3899529824
    deliveryPose1.position.z = 0.0
    deliveryPose1.orientation.x = 0.0
    deliveryPose1.orientation.y = 0.0
    deliveryPose1.orientation.z = -0.769828618017
    deliveryPose1.orientation.w = 0.638250655214

    # At bookshelf
    bookshelfPose1 = Pose()
    bookshelfPose1.position.x = -3.66569411557
    bookshelfPose1.position.y = -14.5338443433
    bookshelfPose1.position.z = 0.0
    bookshelfPose1.orientation.x = 0.0
    bookshelfPose1.orientation.y = 0.0
    bookshelfPose1.orientation.z = 0.65705315908
    bookshelfPose1.orientation.w = 0.753844245281
 
    bookshelfPose2 = Pose()
    bookshelfPose2.position.x = -3.55608035699
    bookshelfPose2.position.y = -14.5941557214
    bookshelfPose2.position.z = 0.0
    bookshelfPose2.orientation.x = 0.0
    bookshelfPose2.orientation.y = 0.0
    bookshelfPose2.orientation.z = 0.611408792497
    bookshelfPose2.orientation.w = 0.791314911054

    bookshelfPose3 = Pose()
    bookshelfPose3.position.x = -3.60172798139
    bookshelfPose3.position.y = -14.7564795155
    bookshelfPose3.position.z = 0.0
    bookshelfPose3.orientation.x = 0.0
    bookshelfPose3.orientation.y = 0.0
    bookshelfPose3.orientation.z = 0.69354384818
    bookshelfPose3.orientation.w = 0.720414415911

    bookshelfPose4 = Pose()
    bookshelfPose4.position.x = -3.60172798139
    bookshelfPose4.position.y = -14.7564795155
    bookshelfPose4.position.z = 0.0
    bookshelfPose4.orientation.x = 0.0
    bookshelfPose4.orientation.y = 0.0
    bookshelfPose4.orientation.z = 0.69354384818
    bookshelfPose4.orientation.w = 0.720414415911




    book1 = BookInfo()
    book1.pose = bookshelfPose1
    book1.torso_height = 0.25
    book1.head_pan = 0.0
    book1.head_tilt = 0.55
    book1.book_name = "Theoretical Neuroscience"
    book1.fiducial_number = 13

    #At bookshelf
    book2 = BookInfo()
    book2.pose = bookshelfPose2
    book2.torso_height = 0.25
    book2.head_pan = 0.0
    book2.head_tilt = 0.55
    book2.book_name = "Introduction to Robotics"
    book2.fiducial_number = 4

    book3 = BookInfo()
    book3.pose = bookshelfPose3
    book3.torso_height = 0.25
    book3.head_pan = 0.0
    book3.head_tilt = 0.55
    book3.book_name = "Praise Justin"
    book3.fiducial_number = 5

    book4 = BookInfo()
    book4.pose = bookshelfPose4
    book4.torso_height = 0.25
    book4.head_pan = 0.0
    book4.head_tilt = 0.55 
    book4.book_name = "Praise Justin"
    book4.fiducial_number = 3

    home1 = BookInfo()
    home1.pose = homePose1
    home1.torso_height = 0.0
    home1.head_pan = 0.0
    home1.head_tilt = 0.0
    home1.book_name = "Praise Justin"
    home1.fiducial_number = 0

    delivery1 = BookInfo()
    delivery1.pose = deliveryPose1
    delivery1.torso_height = 0.0
    delivery1.head_pan = 0.0
    delivery1.head_tilt = 0.0
    delivery1.book_name = "Praise Justin"
    delivery1.fiducial_number = 0

    # home2 = BookInfo()
    # home2.pose = home2
    # home2.torso_height = 0.4
    # home2.head_pan = -0.15
    # home2.head_tilt = 0.3
    # home2.book_name = "Praise Justin"
    # home2.fiducial_number = 13

    # home3 = BookInfo()
    # home3.pose = home3
    # home3.torso_height = 0.4
    # home3.head_pan = -0.15
    # home3.head_tilt = 0.3
    # home3.book_name = "Praise Justin"
    # home3.fiducial_number = 13

    # home4 = BookInfo()
    # home4.pose = home4
    # home4.torso_height = 0.4
    # home4.head_pan = -0.15
    # home4.head_tilt = 0.3
    # home4.book_name = "Praise Justin"
    # home4.fiducial_number = 13



    #books = {1: book1, 2: book2, 3: book3, 4: book4, -1: home1, -2: home2, -3: home3, -4: home4}
    books = {1: book1, 2: book2, 3: book3, 4: book4, -1: home1, -2: delivery1}
    bookPath = "/home/team4/catkin_ws/src/cse481c/library_bot/nodes/book_database_real_robot.p"

print books
pickle.dump(books, open(bookPath, "wb"))

# # Test to make sure reading from pickle file works

# pickle_test = pickle.load(open("./book_database.p", "rb"))
# print pickle_test[1].torso_height

# The torso_height, head_pan, and head_tilt assume the point cloud "topShelfNoHand.bag"