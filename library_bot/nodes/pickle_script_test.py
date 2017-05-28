import pickle
from database_reader import BookInfo
from geometry_msgs.msg import Pose

# At bookshelf
bookshelfPose = Pose()
bookshelfPose.position.x = 2.56
bookshelfPose.position.y = 3.22
bookshelfPose.position.z = 0.0
bookshelfPose.orientation.x = 0.0
bookshelfPose.orientation.y = 0.0
bookshelfPose.orientation.z = 0.016551
bookshelfPose.orientation.w = 1.0


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

books = {1: book1, 2: book2, 3: book3}

print books
pickle.dump(books, open("./book_database.p", "wb"))

# # Test to make sure reading from pickle file works

# pickle_test = pickle.load(open("./book_database.p", "rb"))
# print pickle_test[1].torso_height

# The torso_height, head_pan, and head_tilt assume the point cloud "topShelfNoHand.bag"