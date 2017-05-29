from main_runner import BookServer

def main():
    inpt = None
    while True:
        print "Type: (home) for the home positions, x: [1, 4]."
        print "Type: (bookShelf) for the bookshelf positions, y: [1, 4]"
        inpt = raw_input()
        bookServ = BookServer()
        if inpt == "home":
            print "type number 1-4"
            inpt = int(raw_input)
            book_id = -inpt

        else:
            print "type number 1-4"
            inpt = int(raw_input)
            book_id = inpt

if __name__ == '__main__':
    main()