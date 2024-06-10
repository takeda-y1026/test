import cv2
import os

script_directory = os.path.dirname(os.path.abspath(__file__))
cascade_path = os.path.join(script_directory, '../data/haarcascade_frontalface_alt.xml')

def main(): #リアルタイム読み込み
    
    capture=cv2.VideoCapture(0)
    cascade=cv2.CascadeClassifier(cascade_path)
    if capture.isOpened() is False:
        raise("IO Error")
    
    while True:
        ret,img=capture.read()
        if ret == False:
            continue

        facerect=cascade.detectMultiScale(img)

        if len(facerect)>0:
            for rect in facerect:
                cv2.rectangle(img,tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]),(0,0,255),thickness=2)
        else:
            print("no face")
        
#カメラの映像を表示
        cv2.imshow("window1",img)
        k=cv2.waitKey(10)
        if k==ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
