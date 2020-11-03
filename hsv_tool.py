import cv2 as cv
class Hsv_change:
    #hsv 调试
    def __init__(self):
        self.i1 = 0
        self.i2 = 0
        self.i3 = 0
        self.i4 = 0
        self.i5 = 0
        self.i6 = 0
        pass
 
    def onChange(self,a, b, hsv):
        if a == 1:
            self.i1 = b
        elif a == 2:
            self.i2 = b
        elif a == 3:
            self.i3 = b
        elif a == 4:
            self.i4 = b
        elif a == 5:
            self.i5 = b
        elif a == 6:
            self.i6 = b
        lowerb = (self.i1, self.i2, self.i3)
        upperb = (self.i4, self.i5, self.i6)
        mask = cv.inRange(hsv, lowerb, upperb)
        cv.imshow("mask display", mask)
        print(self.i1, self.i2, self.i3, self.i4, self.i5, self.i6)
 
 
    ## 调用次方法,传入文件路径
    def inRange_byFilePath(self,pathFile=""):
        # img = cv2.imread("data2/test_aubo2.png")
        # img_copy=img.copy()
        dstImgggg = cv.imread(pathFile)
        assert dstImgggg is not None
        img_copysize = cv.resize(dstImgggg, (int(dstImgggg.shape[1] / 2), int(dstImgggg.shape[0] / 2)))
        cv.imshow("img_copysize", img_copysize)
        hsv = cv.cvtColor(img_copysize, cv.COLOR_BGR2HSV)
        cv.imshow("hsv", hsv)
        cv.namedWindow("mask")
        cv.createTrackbar("h1", "mask", 0, 255, lambda a:self.onChange(1, a, hsv))
        cv.createTrackbar("s1", "mask", 0, 255, lambda a:self.onChange(2, a, hsv))
        cv.createTrackbar("v1", "mask", 0, 255, lambda a:self.onChange(3, a, hsv))
        cv.createTrackbar("h2", "mask", 0, 255, lambda a:self.onChange(4, a, hsv))
        cv.createTrackbar("s2", "mask", 0, 255, lambda a:self.onChange(5, a, hsv))
        cv.createTrackbar("v2", "mask", 0, 255, lambda a:self.onChange(6, a, hsv), )
        cv.waitKey(0)
        cv.destroyAllWindows()
 
        ## 调用次方法,传入图片
    def inRange_byImage(self, dstImgggg):
            img_copysize = cv.resize(dstImgggg, (int(dstImgggg.shape[1] / 2), int(dstImgggg.shape[0] / 2)))
            cv.imshow("img_copysize", img_copysize)
            hsv = cv.cvtColor(img_copysize, cv.COLOR_BGR2HSV)
            cv.imshow("hsv", hsv)
            cv.namedWindow("mask")
            cv.createTrackbar("h1", "mask", 0, 255, lambda a: self.onChange(1, a, hsv))
            cv.createTrackbar("s1", "mask", 0, 255, lambda a: self.onChange(2, a, hsv))
            cv.createTrackbar("v1", "mask", 0, 255, lambda a: self.onChange(3, a, hsv))
            cv.createTrackbar("h2", "mask", 0, 255, lambda a: self.onChange(4, a, hsv))
            cv.createTrackbar("s2", "mask", 0, 255, lambda a: self.onChange(5, a, hsv))
            cv.createTrackbar("v2", "mask", 0, 255, lambda a: self.onChange(6, a, hsv), )
            cv.waitKey(0)
            cv.destroyAllWindows()
 
 
 
 
if __name__ == '__main__':
    ##使用:
    h=Hsv_change()
    h.inRange_byFilePath("chest/stage.jpg")
