import cv2
from math import pi
import numpy as np

try:
    from PIL import Image
except ImportError:
    import Image
import pytesseract

#pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

POLY_APPROX_COEFF = 0.03

RECT_MIN_AR = 1 #min aspect ratio
RECT_MAX_AR = 2.5 #max aspect ratio
RECT_MIN_AREA = 500
RECT_MAX_AREA = 10000
RECT_MIN_BBOX_FILL = 0.1 #min pct that rect contour fills its minimal bounding box

def degToRad(degrees):
    return degrees * pi / 180

def order_points(pts):
	# initialzie a list of coordinates that will be ordered
	# such that the first entry in the list is the top-left,
	# the second entry is the top-right, the third is the
	# bottom-right, and the fourth is the bottom-left
	rect = np.zeros((4, 2), dtype = "float32")
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
	# return the ordered coordinates
	return rect

def four_point_transform(image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
	# compute the perspective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
	# return the warped image
	return warped

class PlateRect:
    def __init__(self, contour, frame):
        self.perimeter = cv2.arcLength(contour, True)
        self.frame = frame
        self.contour = contour
        self.contourArea = cv2.contourArea(self.contour)
        self.boundingBox = cv2.minAreaRect(contour)
        self.boundingBoxContour = np.int0(cv2.boxPoints(self.boundingBox))
        self.center = self.boundingBox[0]
        self.dims = self.boundingBox[1]
        self.aspectRatio = self.dims[0] / self.dims[1]
        self.boundingBoxArea = self.dims[0] * self.dims[1]
        self.angle = -1 * self.boundingBox[2]

        if self.aspectRatio < 1:
            self.aspectRatio = 1 / self.aspectRatio
            self.angle += 90

        self.angle = degToRad(self.angle)

    def perspectiveTransform(self):
        persFrame = cv2.resize(four_point_transform(self.frame, self.contour.reshape(4,2)), (260,120))
        persFrame = persFrame[5:116, 5:256]
        _, self.threshedPersFrame = cv2.threshold(persFrame,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        return self.threshedPersFrame

    def ocrFrame(self):
        img_rgb = cv2.cvtColor(self.threshedPersFrame, cv2.COLOR_GRAY2RGB)
        self.ocrStr = pytesseract.image_to_string(img_rgb).encode("ascii", "ignore")
        return self.ocrStr

def auto_canny(image, sigma=.8):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
	# return the edged image
	return edged

def threshImg(grayImg, exposure = -2, kernel_size = 75):
    #blurImg = cv2.blur(grayImg, (3,3))
    # threshedImg = cv2.adaptiveThreshold(
    #     grayImg, 
    #     255, 
    #     cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
    #     cv2.THRESH_BINARY, 
    #     kernel_size, 
    #     exposure
    # ) 
    #_, threshedImg = cv2.threshold(grayImg, 95, 255, cv2.THRESH_BINARY_INV)
    #kernel = np.ones((3,3), np.uint8) 
    #dilatedImg = cv2.dilate(threshedImg, kernel, iterations=1)
    #threshedImg = auto_canny(blurImg, sigma=0.1)
    threshedImg = cv2.Canny(grayImg, 35, 70)
    return threshedImg

def simplifyContour(contour, simplifyCoeff = POLY_APPROX_COEFF):
    perimeter = cv2.arcLength(contour, True)
    approxCnt = cv2.approxPolyDP(contour, simplifyCoeff*perimeter, True)
    return approxCnt

def getPlates(frame):
    greyFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshedImg = threshImg(greyFrame)

    img, contours, hierarchy = cv2.findContours(threshedImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    
    for contour in contours:
        approxCnt = simplifyContour(contour)
        if len(approxCnt) == 4:
            plate = PlateRect(approxCnt, greyFrame)
            if all((
                RECT_MIN_AREA <= plate.contourArea <= RECT_MAX_AREA,
                RECT_MIN_AR < plate.aspectRatio < RECT_MAX_AR,
                plate.contourArea / plate.boundingBoxArea >= RECT_MIN_BBOX_FILL
            )):
                rects.append(plate)
                print(plate.contourArea)

    return (
        sorted(rects, key=lambda rect: rect.contourArea), 
        cv2.cvtColor(threshedImg, cv2.COLOR_GRAY2BGR)
    )