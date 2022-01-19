
from mimetypes import init


class test:
    def __init__(self):
        self.lista = []
    def process(self):
        listB = []
        for x in range(10):
            self.lista.append(x)
            listB.append(self.lista[:])
            
        print(listB)

obj = test()

obj.process()
print("OK")