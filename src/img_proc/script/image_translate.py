import cv2

def main(): #リアルタイム読み込み
    
    capture=cv2.VideoCapture(0)
    height,width=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    center,degree=(width//2,height//2),0.0 #中心,回転角
    if capture.isOpened() is False:
        raise("IO Error")
    
    while True:
        ret,img=capture.read()
        if ret == False:
            break

        #回転
        affin_trans=cv2.getRotationMatrix2D(center,degree,1.0)
        img_r=cv2.warpAffine(img,affin_trans,(width,height))
        degree+=0.15

        #モザイク処理
        SCALE=0.05 #縮小率
        img_s=cv2.resize(img,(round(width*SCALE),round(height*SCALE)),interpolation=cv2.INTER_NEAREST)
        img_m=cv2.resize(img_s,(width,height),interpolation=cv2.INTER_NEAREST)

        #グレースケール変換処理
        gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #エッジ検出
        img_e=cv2.Canny(gray_frame,40.0,200.0)

        #2値化処理
        n,binary_frame = cv2.threshold(gray_frame, 128, 255, cv2.THRESH_BINARY)
        
#カメラの映像を表示 見たい映像のコメントアウトを外す
        cv2.imshow("window",img)
        cv2.imshow("window1",img_r)
        cv2.imshow("window2",img_e)
        cv2.imshow("window3",img_s)
        cv2.imshow("window4",img_m)
        cv2.imshow("window5",gray_frame)
        cv2.imshow("window6",binary_frame)
        k=cv2.waitKey(10)
        if k==ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()