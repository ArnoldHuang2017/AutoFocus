import traceback

# -- coding: utf-8 -
class Queue(object):
    def __init__(self, size):
        self.data_list = []
        self.__size = size
        self.__cur = 0

    def IsFull(self):
        return self.__cur == self.__size

    def IsEmpty(self):
        return self.__cur == 0

    def PushBack(self, data):
        if not self.IsFull():
            self.data_list.append(data)
            self.__cur += 1
        else:
            #print("The queue is full!")
            pass

    def PushFront(self, data):
        if self.IsEmpty():
            self.PushBack(data)
        else:
            temp = [data]
            temp.extend(self.data_list)
            self.data_list = temp
        self.__cur += 1

    def PopFront(self):
        # print(f"Before pop front:{self.data_list}")
        if not self.IsEmpty():
            self.data_list.pop(0)
            # print(f"After pop front:{self.data_list}")
        else:
            print("The queue is empty")

        self.__cur -= 1

        return self.data_list

    def PopBack(self):
        if not self.IsEmpty():
            self.data_list.pop(-1)

        self.__cur -= 1

        return self.data_list

    def Size(self):
        return self.__cur

    def Back(self):
        if not self.IsEmpty():
            return self.data_list[-1]
        return None

    def UpdateBack(self, data):
        if self.IsEmpty():
            return

        self.data_list[-1] = data

    def Clear(self):
        self.data_list = []
        self.__cur = 0

    def Show(self):
        print(self.data_list)

    def GetIndex(self, index):
        if not self.IsEmpty():
            return self.data_list[index]
