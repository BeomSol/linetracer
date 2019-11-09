import numpy as np
import cv2
import time

class ORB:
    MIN_MATCH_COUNT = 10
    FLANN_INDEX_LSH = 6
#    index_params = search_params = flann = stime = 0
#    orb = obj_index = mask = filter_type = 0
    obj_size = []; obj_name = []; kp = []; des = []
    
    def __init__(self,feature_num,threshold,_type,_blur):
        #parameters for FLANN
        self.index_params= dict(algorithm = self.FLANN_INDEX_LSH,   
                        table_number = 6, # 12
                        key_size = 12, # 20
                        multi_probe_level = 1) #2
        self.search_params = dict(checks=50) # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        self.filter = _type
        self.mask = _blur
        self.MIN_MATCH_COUNT = threshold
        self.orb = cv2.ORB_create(feature_num)      # Initiate detector
        self.orb_obj = cv2.ORB_create(int(feature_num/3))      # Initiate detector

        self.obj_index = 0

    def reduce_noise(self,img_o):           #filtering
        if self.filter == 'G':
            return cv2.GaussianBlur(img_o,self.mask,0)
        elif self.filter == 'B':
            tmp = self.mask[0]
            return cv2.bilateralFilter(img_o,tmp,tmp*5-4,tmp*5-4)
        else:
            return cv2.blur(img_o,self.mask)

    def add_object(self,object_name,object_img):
        #make gray and blurring
        try:        
            obj = cv2.cvtColor(object_img,cv2.COLOR_BGR2GRAY)
        except:
            obj = object_img
            
        obj_filter = self.reduce_noise(obj)
        
        self.obj_size.append(obj)       #save object size for make rect
        
        tmp_kp, tmp_des = self.orb_obj.detectAndCompute(obj_filter,None)
        self.kp.append(tmp_kp); self.des.append(tmp_des)    #save several keypoint, descriper
        
        obj = cv2.drawKeypoints(obj,self.kp[self.obj_index],None, flags=0)
        self.obj_name.append(object_name)         #save object name
        self.obj_index += 1
        
        cv2.imshow(object_name,obj)             #show objects

    def matching(self,main_img,down_scale=1,draw = True):
        if not self.obj_index:                  #if no object, return
            print('Please make object picture.')
            return
        
        self.stime = time.time()

        #Down scaling for save time
        rows,cols,chs = main_img.shape
        main_img2 = main_img
        main_img2 = cv2.resize(main_img,(int(cols/down_scale),int(rows/down_scale)))

        #Make gray and blurring
        main = cv2.cvtColor(main_img2, cv2.COLOR_BGR2GRAY)
        main_filter = self.reduce_noise(main)
        
        m_kp, m_des = self.orb.detectAndCompute(main_filter,None)
        main_kp = cv2.drawKeypoints(main_img2,m_kp,None, flags=0)
        
        if draw:
            main_kp = cv2.drawKeypoints(main_img,m_kp,None, flags=0)

        matches = []
        matched_name = []
        matched_good = []
        
        for i in range(self.obj_index):
            good = []
            
            try:
                matches.append(self.flann.knnMatch(self.des[i],m_des,k=2))  #Match with several objects
            
                for m,n in matches[i]:          # ratio test as per Lowe's paper
                    if m.distance < 0.7*n.distance:
                        good.append(m)
            except:
                pass
                
            if len(good) >= self.MIN_MATCH_COUNT:   # Over threshold = success match
                matched_name.append(self.obj_name[i])
                matched_good.append(len(good))
                    
                
                #Draw rect on matched objects
                if draw:
                    src_pts = np.float32([ self.kp[i][m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                    dst_pts = np.float32([ m_kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                    try:  
                        h,w = self.obj_size[i].shape
                        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
                        dst = cv2.perspectiveTransform(pts,M)
                    except:
                        continue
                        
                    main_img = cv2.polylines(main_img,[np.int32(dst*2)],True,255,3, cv2.LINE_AA)
            else:   #Fail match
                #print ("Not enough matches are found - %d/%d" %(len(good),self.MIN_MATCH_COUNT))
                pass

        if len(matched_name) > 1:
            matched = matched_name[matched_good.index(max(matched_good))]
        elif len(matched_name) == 1:
            matched = matched_name[0]
        else :
            matched = 'none'
            
            
        cv2.imshow('main',main_img)
        
        return matched
        
    
    def spend_time(self):
        t = int((time.time()-self.stime)*1000)
        print('Spend time : {} ms'.format(t))

cap = cv2.VideoCapture(0)
running = True

#Make ORB class(initiate detector,filter_type ,threshold, blur_mask)
#filter_type == 'G'(Gaussian),'B'(Bilateral), other char(blur)
orb = ORB(300,7,'G',(5,5))

#Read Object Pictures
obj1 = cv2.imread('stop.jpg')
obj2 = cv2.imread('slow.jpg')
obj3 = cv2.imread('noentry.jpg')

#Add Object Pictiures
orb.add_object('stop',obj1)
orb.add_object('slow',obj2)
orb.add_object('noentry',obj3)

while running:
    ret, frame = cap.read()
##    if not ret:
##        break
    
    print(orb.matching(frame,2,False))     #matching(Camera frame, down_scale, draw rect)

    orb.spend_time()                #How many spend times for match
    k = cv2.waitKey(1) & 0xFF
    if k == 27:                     #exit by ESC
        running =False

        
        
cv2.destroyAllWindows()
cap.release()
