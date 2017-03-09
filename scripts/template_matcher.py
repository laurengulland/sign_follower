import cv2
from scipy.linalg import norm
import numpy as np
from math import isnan

"""
This code determines which of a set of template images matches
an input image the best using the SIFT algorithm
"""

class TemplateMatcher(object):

    def __init__ (self, images, min_match_count=10, good_thresh=0.7):
        self.signs = {} #maps keys to the template images
        self.kps = {} #maps keys to the keypoints of the template images
        self.descs = {} #maps keys to the descriptors of the template images
        if cv2.__version__=='3.1.0-dev':
            self.sift = cv2.xfeatures2d.SIFT_create()
        else:
            self.sift = cv2.SIFT() #initialize SIFT to be used for image matching
        self.ransac_thresh = 0.5
        # for potential tweaking
        self.min_match_count = min_match_count
        self.good_thresh = good_thresh #use for keypoint threshold
        for k, filename in images.iteritems():
            # load template sign images as grayscale
            self.signs[k] = cv2.imread(filename,0)
            # precompute keypoints and descriptors for the template sign
            self.kps[k], self.descs[k] = self.sift.detectAndCompute(self.signs[k],None)

    def predict(self, img):
        """
        Uses gather predictions to get visual diffs of the image to each template
        returns a dictionary, keys being signs, values being confidences
        """
        visual_diff = {}
        template_confidence={}

        kp, des = self.sift.detectAndCompute(img,None)

        for k in self.signs.keys():
            #cycle trough templage images (k) and get the image differences
            visual_diff[k] = self._compute_prediction(k, img, kp, des)

        if visual_diff: #check if None
            val_total = sum(visual_diff.values())
            # print 'val total', val_total
            #BUG: "ValueError: operands could not be broadcast together with shapes (93,98) (82,100"

            for k in visual_diff:
                visual_diff[k] /= val_total

        else: # if visual diff was not computed (bad crop, homography could not be computed)
            # set 0 confidence for all signs
            template_confidence = {k: 0 for k in self.signs.keys()}

        return template_confidence


    def _compute_prediction(self, k, img, kp, des):
        """
        Return comparison values between a template k and given image
        k: template image for comparison, img: scene image
        kp: keypoints from scene image,   des: descriptors from scene image
        """

        # TODO: find corresponding points in the input image and the template image
        #       put keypoints from template image in template_pts
        #       put corresponding keypoints from input image in img_pts

        kp,des = self.sift.detectAndCompute(self.signs[k],None)
        img_kp, img_des = self.sift.detectAndCompute(img,None)


        # Brute Force Matcher with default params - get 'k=2' best matches
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des,img_des, k=2)

        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)

        # for m in good:
        #     print good.index(m)
        #     print img_kp
        #     print img_kp[k]
        #     print img_kp[k][good.index(m)]
        # img_pts = [img_kp[good.index(m)] for m in good]
        # template_pts = [kp[good.index(m)] for m in good]
        img_pts = np.float32([kp[m.queryIdx].pt for m in good])
        img_pts.reshape(-1,1,2)
        template_pts = np.float32([img_kp[m.trainIdx].pt for m in good])
        template_pts.reshape(-1,1,2)

        # Transform input image so that it matches the template image as well as possible
        M, mask = cv2.findHomography(np.array(img_pts), np.array(template_pts), cv2.RANSAC, self.ransac_thresh)
        img_T = cv2.warpPerspective(img, M, self.signs[k].shape[::-1])

        visual_diff = compare_images(img_T, self.signs[k])
        return visual_diff
# end of TemplateMatcher class

def compare_images(img1, img2):
    #normalize images
    # print 'np  ', np.std(img1)
    # print '.std', img1.std()

    err = np.sum((img1.astype('float') - img2.astype('float')) ** 2)
    err /= float(img1.shape[0] * img2.shape[1])
    # print err
    return err

    # img1 = (img1-img1.mean())/img1.std()
    # img2 = (img2-img2.mean())/img2.std()
    # # img1 = normalize(img1)
    # # img2 = normalize(img2)
    # #find mathematical difference between images
    # diff = img1-img2
    # # norms = norm(diff.ravel(),1)
    # return diff
