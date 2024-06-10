import cv2
import os

script_directory = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_directory, '../image/lena.jpg')

def main():
    img=cv2.imread(image_path) #画像読み込み
    #BGR→RGBへの変換
    img_rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    #画像の表示
    cv2.imshow("BGR",img)
    cv2.imshow("RGB",img_rgb)

#それぞれの画像の画素値　(x,y)=(100,100)の画素
    # B_1,G_1,R_1 = img[100, 100, 0],img[100, 100, 1],img[100, 100, 2] #4行目の画像のBGRの値
    # R_2,G_2,B_2 = img_rgb[100, 100, 0],img_rgb[100, 100, 1],img_rgb[100, 100, 2] #6行目の画像のRGBの値
    # print(B_1,G_1,R_1) #B=87 G=74 R=182
    # print(R_2,G_2,B_2)  #R=182 G=74 B=87 9行目のimshowではBGR=(182,74,87)してるから青っぽい

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()