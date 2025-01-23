package com.femto.circle.utily;

import android.graphics.Bitmap;
import android.util.Log;
import java.nio.ByteBuffer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class ConvertMat {
  public byte Center_Operate_Mode = 0;
  
  private int Change_Decode_Vedio_Start_Line_Num;
  
  private int Decode_One_Picture_Operate_Size;
  
  private int Decode_One_Picture_Target_Size;
  
  private int Decode_Vedio_Height;
  
  private boolean Decode_Vedio_Lower_Level_Flag;
  
  private int Decode_Vedio_Start_Line_Num;
  
  private int Decode_Vedio_Width;
  
  public byte Exchange_Mode = 0;
  
  private byte[] Gamma_Code = new byte[256];
  
  private Boolean Jpeg_Version_Flag = Boolean.valueOf(false);
  
  private int Left_Mat_Width;
  
  private int[] Lightness_Average_Serial_Array = new int[2048];
  
  private int Middle_Mat_Heigh;
  
  private int Middle_Mat_Width;
  
  private int Right_Mat_Heigh;
  
  private int Right_Mat_Width;
  
  private int Swift_Width_Num;
  
  private int Switch_Serial_Initial_Num;
  
  private int Switch_Serial_Num_Add_Five;
  
  private int Switch_Serial_Num_Add_Four;
  
  private int Switch_Serial_Num_Add_One;
  
  private int Switch_Serial_Num_Add_Seven;
  
  private int Switch_Serial_Num_Add_Six;
  
  private int Switch_Serial_Num_Add_Three;
  
  private int Switch_Serial_Num_Add_Two;
  
  byte[] Target_Bin;
  
  private int Target_Result_Width;
  
  private int Target_Result_Width_Eighth;
  
  private byte[] Three_Fan_File_Formats_Bin;
  
  private boolean initial_rang_flag;
  
  private boolean isFirstPhotoInited;
  
  private Rect selectedRect;
  
  private int startHeight;
  
  private int startwidth;
  
  private int startx;
  
  private int starty;
  
  public float targetFPS = 12.0F;
  
  private float totalScale;
  
  private Point userPoint;
  
  private float userRadius;
  
  private byte[] formatJpegData(byte[] paramArrayOfbyte) {
    int i = paramArrayOfbyte.length;
    int j = i % 4;
    int k = j;
    if (j > 0)
      k = 4 - j; 
    j = i + k;
    byte b1 = (byte)(j >> 24 & 0xFF);
    byte b2 = (byte)(j >> 16 & 0xFF);
    byte b3 = (byte)(j >> 8 & 0xFF);
    byte b4 = (byte)(j & 0xFF);
    byte[] arrayOfByte = new byte[k];
    ByteBuffer byteBuffer = ByteBuffer.allocate(paramArrayOfbyte.length + 4 + k);
    byteBuffer.put(new byte[] { b4, b3, b2, b1 });
    byteBuffer.put(paramArrayOfbyte);
    byteBuffer.put(arrayOfByte);
    return byteBuffer.array();
  }
  
  public float Get_Current_FPS() {
    return this.targetFPS;
  }
  
  public void Rotate_Decode_Vedio_Start_Line_Num(int paramInt) {}
  
  //�� ��ȣ�� ������� �ʱ� ����
  public void Set_Device_Config(int paramInt) {
    boolean bool1 = false;
    boolean bool2 = false;
    boolean bool3 = false;
    boolean bool4 = false;
    boolean bool5 = false;
    boolean bool6 = false;
    boolean bool7 = false;
    boolean bool8 = false;
    boolean bool9 = false;
    boolean bool10 = false;
    boolean bool11 = false;
    boolean bool12 = false;
    boolean bool13 = false;
    boolean bool14 = false;
    boolean bool15 = false;
    boolean bool16 = false;
    boolean bool17 = false;
    boolean bool18 = false;
    boolean bool19 = false;
    boolean bool20 = false;
    boolean bool21 = false;
    this.Jpeg_Version_Flag = Boolean.valueOf(false);
    Boolean bool = Boolean.valueOf(true);
    switch (paramInt) {
      default:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        return;
      case 27:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Width = 256;
        this.Decode_Vedio_Height = 1056;
        this.Left_Mat_Width = 192;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 704;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 352;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 147;
        return;
      case 26:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Width = 224;
        this.Decode_Vedio_Height = 1152;
        this.Left_Mat_Width = 160;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 576;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 288;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 156;
        return;
      case 25:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Width = 192;
        this.Decode_Vedio_Height = 1536;
        this.Left_Mat_Width = 128;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 768;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 384;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 200;
        return;
      case 24:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 18.0F;
        this.Decode_Vedio_Width = 192;
        this.Decode_Vedio_Height = 1152;
        this.Left_Mat_Width = 128;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 576;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 288;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 160;
        return;
      case 23:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 20.0F;
        this.Decode_Vedio_Width = 160;
        this.Decode_Vedio_Height = 1056;
        this.Left_Mat_Width = 96;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 528;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 352;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        return;
      case 22:
        this.Jpeg_Version_Flag = bool;
        this.targetFPS = 24.0F;
        this.Decode_Vedio_Width = 128;
        this.Decode_Vedio_Height = 1024;
        this.Left_Mat_Width = 64;
        this.Middle_Mat_Width = 32;
        this.Middle_Mat_Heigh = 512;
        this.Right_Mat_Width = 32;
        this.Right_Mat_Heigh = 256;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        return;
      case 21:
        this.Center_Operate_Mode = 1;
        this.Exchange_Mode = 12;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 160;
        this.Decode_Vedio_Height = 1080;
        this.Target_Result_Width = 480;
        this.Swift_Width_Num = 160 / 8;
        this.Switch_Serial_Initial_Num = 480 / 8 - 1;
        paramInt = 480 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 518400;
        this.Decode_One_Picture_Target_Size = 518400;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool21; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 20:
        this.Center_Operate_Mode = 1;
        this.Exchange_Mode = 7;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 256;
        this.Decode_Vedio_Height = 1020;
        this.Target_Result_Width = 768;
        this.Swift_Width_Num = 256 / 8;
        this.Switch_Serial_Initial_Num = 768 / 8 - 1;
        paramInt = 768 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 783360;
        this.Decode_One_Picture_Target_Size = 783360;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 657408;
        for (paramInt = bool1; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 19:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 11;
        this.targetFPS = 8.0F;
        this.Decode_Vedio_Lower_Level_Flag = false;
        this.Decode_Vedio_Width = 48;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 108;
        this.Swift_Width_Num = 48 / 8;
        this.Switch_Serial_Initial_Num = 108 / 6 - 1;
        paramInt = 108 / 6;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 73728;
        this.Decode_One_Picture_Target_Size = 55296;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool2; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt / 3 << 2; 
        return;
      case 18:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 10;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 48;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 144;
        this.Swift_Width_Num = 48 / 8;
        this.Switch_Serial_Initial_Num = 144 / 8 - 1;
        paramInt = 144 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 73728;
        this.Decode_One_Picture_Target_Size = 73728;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool3; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 17:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 9;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 64;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 192;
        this.Swift_Width_Num = 64 / 8;
        this.Switch_Serial_Initial_Num = 192 / 8 - 1;
        paramInt = 192 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 98304;
        this.Decode_One_Picture_Target_Size = 98304;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool4; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 16:
        this.Center_Operate_Mode = 2;
        this.Exchange_Mode = 7;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 256;
        this.Decode_Vedio_Height = 1020;
        this.Target_Result_Width = 768;
        this.Swift_Width_Num = 256 / 8;
        this.Switch_Serial_Initial_Num = 768 / 8 - 1;
        paramInt = 768 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 783360;
        this.Decode_One_Picture_Target_Size = 783360;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 657408;
        for (paramInt = bool5; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 15:
        this.Center_Operate_Mode = 1;
        this.Exchange_Mode = 6;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 224;
        this.Decode_Vedio_Height = 1152;
        this.Target_Result_Width = 672;
        this.Swift_Width_Num = 224 / 8;
        this.Switch_Serial_Initial_Num = 672 / 8 - 1;
        paramInt = 672 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 774144;
        this.Decode_One_Picture_Target_Size = 774144;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 649152;
        for (paramInt = bool6; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 14:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 4;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = false;
        this.Decode_Vedio_Width = 128;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 288;
        this.Swift_Width_Num = 128 / 8;
        this.Switch_Serial_Initial_Num = 288 / 6 - 1;
        paramInt = 288 / 6;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 196608;
        this.Decode_One_Picture_Target_Size = 147456;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool7; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt / 3 << 2; 
        return;
      case 13:
        this.Center_Operate_Mode = 1;
        this.Exchange_Mode = 3;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 192;
        this.Decode_Vedio_Height = 1536;
        this.Target_Result_Width = 576;
        this.Swift_Width_Num = 192 / 8;
        this.Switch_Serial_Initial_Num = 576 / 8 - 1;
        paramInt = 576 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 884736;
        this.Decode_One_Picture_Target_Size = 884736;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 741888;
        for (paramInt = bool8; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 12:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 2;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 112;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 336;
        this.Swift_Width_Num = 112 / 8;
        this.Switch_Serial_Initial_Num = 336 / 8 - 1;
        paramInt = 336 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 172032;
        this.Decode_One_Picture_Target_Size = 172032;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool9; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 11:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 1;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = false;
        this.Decode_Vedio_Width = 112;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 252;
        this.Swift_Width_Num = 112 / 8;
        this.Switch_Serial_Initial_Num = 252 / 6 - 1;
        paramInt = 252 / 6;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 172032;
        this.Decode_One_Picture_Target_Size = 129024;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool10; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt / 3 << 2; 
        return;
      case 10:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 192;
        this.Decode_Vedio_Height = 1152;
        this.Target_Result_Width = 576;
        this.Swift_Width_Num = 192 / 8;
        this.Switch_Serial_Initial_Num = 576 / 8 - 1;
        paramInt = 576 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 663552;
        this.Decode_One_Picture_Target_Size = 663552;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 419328;
        for (paramInt = bool11; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 9:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 128;
        this.Decode_Vedio_Height = 1024;
        this.Target_Result_Width = 384;
        this.Swift_Width_Num = 128 / 8;
        this.Switch_Serial_Initial_Num = 384 / 8 - 1;
        paramInt = 384 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 393216;
        this.Decode_One_Picture_Target_Size = 393216;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool12; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 8:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 224;
        this.Decode_Vedio_Height = 1152;
        this.Target_Result_Width = 672;
        this.Swift_Width_Num = 224 / 8;
        this.Switch_Serial_Initial_Num = 672 / 8 - 1;
        paramInt = 672 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 774144;
        this.Decode_One_Picture_Target_Size = 774144;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 649152;
        for (paramInt = bool13; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 7:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 136;
        this.Decode_Vedio_Height = 1024;
        this.Target_Result_Width = 408;
        this.Swift_Width_Num = 136 / 8;
        this.Switch_Serial_Initial_Num = 408 / 8 - 1;
        paramInt = 408 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 417792;
        this.Decode_One_Picture_Target_Size = 417792;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool14; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 6:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 5;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 80;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 240;
        this.Swift_Width_Num = 80 / 8;
        this.Switch_Serial_Initial_Num = 240 / 8 - 1;
        paramInt = 240 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 122880;
        this.Decode_One_Picture_Target_Size = 122880;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool15; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 5:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 8;
        this.targetFPS = 8.0F;
        this.Decode_Vedio_Lower_Level_Flag = false;
        this.Decode_Vedio_Width = 64;
        this.Decode_Vedio_Height = 512;
        this.Target_Result_Width = 144;
        this.Swift_Width_Num = 64 / 8;
        this.Switch_Serial_Initial_Num = 144 / 6 - 1;
        paramInt = 144 / 6;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 98304;
        this.Decode_One_Picture_Target_Size = 73728;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool16; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt / 3 << 2; 
        return;
      case 4:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 256;
        this.Decode_Vedio_Height = 1020;
        this.Target_Result_Width = 768;
        this.Swift_Width_Num = 256 / 8;
        this.Switch_Serial_Initial_Num = 768 / 8 - 1;
        paramInt = 768 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 783360;
        this.Decode_One_Picture_Target_Size = 783360;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 657408;
        for (paramInt = bool17; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 3:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 128;
        this.Decode_Vedio_Height = 640;
        this.Target_Result_Width = 384;
        this.Swift_Width_Num = 128 / 8;
        this.Switch_Serial_Initial_Num = 384 / 8 - 1;
        paramInt = 384 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 245760;
        this.Decode_One_Picture_Target_Size = 245760;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 206976;
        for (paramInt = bool18; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 2:
        this.Center_Operate_Mode = 0;
        this.Exchange_Mode = 0;
        this.targetFPS = 12.0F;
        this.Decode_Vedio_Lower_Level_Flag = true;
        this.Decode_Vedio_Width = 192;
        this.Decode_Vedio_Height = 700;
        this.Target_Result_Width = 576;
        this.Swift_Width_Num = 192 / 8;
        this.Switch_Serial_Initial_Num = 576 / 8 - 1;
        paramInt = 576 / 8;
        this.Target_Result_Width_Eighth = paramInt;
        this.Switch_Serial_Num_Add_One = paramInt;
        this.Switch_Serial_Num_Add_Two = paramInt * 2;
        this.Switch_Serial_Num_Add_Three = paramInt * 3;
        this.Switch_Serial_Num_Add_Four = paramInt * 4;
        this.Switch_Serial_Num_Add_Five = paramInt * 5;
        this.Switch_Serial_Num_Add_Six = paramInt * 6;
        this.Switch_Serial_Num_Add_Seven = paramInt * 7;
        this.Decode_One_Picture_Operate_Size = 403200;
        this.Decode_One_Picture_Target_Size = 403200;
        this.Decode_Vedio_Start_Line_Num = 0;
        this.Change_Decode_Vedio_Start_Line_Num = 0;
        for (paramInt = bool19; paramInt < this.Decode_Vedio_Height; paramInt++)
          this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
        return;
      case 1:
        break;
    } 
    this.Center_Operate_Mode = 0;
    this.Exchange_Mode = 0;
    this.targetFPS = 12.0F;
    this.Decode_Vedio_Lower_Level_Flag = true;
    this.Decode_Vedio_Width = 144;
    this.Decode_Vedio_Height = 1020;
    this.Target_Result_Width = 432;
    this.Swift_Width_Num = 144 / 8;
    this.Switch_Serial_Initial_Num = 432 / 8 - 1;
    paramInt = 432 / 8;
    this.Target_Result_Width_Eighth = paramInt;
    this.Switch_Serial_Num_Add_One = paramInt;
    this.Switch_Serial_Num_Add_Two = paramInt * 2;
    this.Switch_Serial_Num_Add_Three = paramInt * 3;
    this.Switch_Serial_Num_Add_Four = paramInt * 4;
    this.Switch_Serial_Num_Add_Five = paramInt * 5;
    this.Switch_Serial_Num_Add_Six = paramInt * 6;
    this.Switch_Serial_Num_Add_Seven = paramInt * 7;
    this.Decode_One_Picture_Operate_Size = 440640;
    this.Decode_One_Picture_Target_Size = 440640;
    this.Decode_Vedio_Start_Line_Num = 0;
    this.Change_Decode_Vedio_Start_Line_Num = 0;
    for (paramInt = bool20; paramInt < this.Decode_Vedio_Height; paramInt++)
      this.Lightness_Average_Serial_Array[paramInt] = this.Target_Result_Width * paramInt; 
  }
  
  public void adjustGamma(int paramInt) {
    boolean bool1 = false;
    boolean bool2 = false;
    int i = 0;
    if (paramInt == 0) {
      for (paramInt = i; paramInt < 256; paramInt = i) {
        byte[] arrayOfByte = this.Gamma_Code;
        i = paramInt + 1;
        arrayOfByte[paramInt] = (byte)(paramInt * i * i >> 16);
      } 
    } else {
      i = bool2;
      if (paramInt == 1) {
        for (paramInt = bool1; paramInt < 256; paramInt = i) {
          byte[] arrayOfByte = this.Gamma_Code;
          i = paramInt + 1;
          arrayOfByte[paramInt] = (byte)(paramInt * i >> 8);
        } 
      } else {
        while (i < 256) {
          if (i > 180) {
            this.Gamma_Code[i] = -1;
          } else {
            this.Gamma_Code[i] = (byte)((i + 1) * i >> 7);
          } 
          i++;
        } 
      } 
    } 
  }
  
  //���� ��Ʈ���� > openCV Mat���� ó�� �� byte[] ����
  public byte[] convert(Bitmap paramBitmap) {
    Mat mat = new Mat();
    Utils.bitmapToMat(paramBitmap, mat);
    int i = (int)(this.selectedRect.size()).width;
    int j = (int)(this.selectedRect.size()).width;
    if (!this.isFirstPhotoInited) {
      double d1 = (i >> 1);
      double d2 = (j >> 1);
      Point point = new Point(d1, d2);
      if (i < j) {
        Imgproc.linearPolar(mat, mat, point, d1, 0);
      } else {
        Imgproc.linearPolar(mat, mat, point, d2, 0);
      } 
      Size size = new Size();
      size.width = this.Decode_Vedio_Width;
      size.height = this.Decode_Vedio_Height;
      this.Three_Fan_File_Formats_Bin = new byte[this.Decode_One_Picture_Target_Size];
      this.Target_Bin = new byte[this.Decode_One_Picture_Operate_Size];
      Imgproc.resize(mat, mat, size);
      if (this.Jpeg_Version_Flag.booleanValue()) {
        Imgproc.cvtColor(mat, mat, 3);
      } else {
        Imgproc.cvtColor(mat, mat, 1);
      } 
      mat.convertTo(mat, CvType.CV_8UC3);
      mat.get(0, 0, this.Target_Bin);
      if (!this.Jpeg_Version_Flag.booleanValue())
        for (i = 0; i < this.Decode_One_Picture_Operate_Size; i++) {
          byte[] arrayOfByte = this.Target_Bin;
          arrayOfByte[i] = this.Gamma_Code[arrayOfByte[i] & 0xFF];
        }  
      if (this.Jpeg_Version_Flag.booleanValue()) {
        Mat mat1;
        Mat mat2;
        Mat mat3;
        Core.flip(mat, mat, -1);
        if (this.Decode_Vedio_Start_Line_Num > 0) {
          i = this.Decode_Vedio_Start_Line_Num;
          Mat mat4 = mat.submat(new Rect(0, i, this.Decode_Vedio_Width, this.Decode_Vedio_Height - i));
          Mat mat5 = mat.submat(new Rect(0, 0, this.Decode_Vedio_Width, this.Decode_Vedio_Start_Line_Num));
          Mat mat6 = Mat.zeros(new Size(this.Decode_Vedio_Width, this.Decode_Vedio_Height), CvType.CV_8UC3);
          mat4.copyTo(mat6.submat(new Rect(0, 0, this.Decode_Vedio_Width, this.Decode_Vedio_Height - this.Decode_Vedio_Start_Line_Num)));
          j = this.Decode_Vedio_Height;
          i = this.Decode_Vedio_Start_Line_Num;
          mat5.copyTo(mat6.submat(new Rect(0, j - i, this.Decode_Vedio_Width, i)));
          mat1 = mat6.submat(new Rect(0, 0, this.Left_Mat_Width, this.Decode_Vedio_Height));
          mat2 = mat6.submat(new Rect(this.Left_Mat_Width, 0, this.Middle_Mat_Width, this.Decode_Vedio_Height));
          Imgproc.resize(mat2, mat2, new Size(this.Middle_Mat_Width, this.Middle_Mat_Heigh));
          mat3 = mat6.submat(new Rect(this.Left_Mat_Width + this.Middle_Mat_Width, 0, this.Right_Mat_Width, this.Decode_Vedio_Height));
          Imgproc.resize(mat3, mat3, new Size(this.Right_Mat_Width, this.Right_Mat_Heigh));
          mat4.release();
          mat5.release();
          mat6.release();
        } else {
          mat1 = mat.submat(new Rect(0, 0, this.Left_Mat_Width, this.Decode_Vedio_Height));
          mat2 = mat.submat(new Rect(this.Left_Mat_Width, 0, this.Middle_Mat_Width, this.Decode_Vedio_Height));
          Imgproc.resize(mat2, mat2, new Size(this.Middle_Mat_Width, this.Middle_Mat_Heigh));
          mat3 = mat.submat(new Rect(this.Left_Mat_Width + this.Middle_Mat_Width, 0, this.Right_Mat_Width, this.Decode_Vedio_Height));
          Imgproc.resize(mat3, mat3, new Size(this.Right_Mat_Width, this.Right_Mat_Heigh));
        } 
        MatOfByte matOfByte3 = new MatOfByte();
        MatOfByte matOfByte1 = new MatOfByte();
        MatOfByte matOfByte2 = new MatOfByte();
        Imgcodecs.imencode(".jpeg", mat3, matOfByte3, new MatOfInt(new int[] { 1, 95 }));
        Imgcodecs.imencode(".jpeg", mat2, matOfByte1, new MatOfInt(new int[] { 1, 95 }));
        Imgcodecs.imencode(".jpeg", mat1, matOfByte2, new MatOfInt(new int[] { 1, 95 }));
        mat3.release();
        mat2.release();
        mat1.release();
        mat.release();
        byte[] arrayOfByte1 = matOfByte3.toArray();
        byte[] arrayOfByte5 = matOfByte1.toArray();
        byte[] arrayOfByte2 = matOfByte2.toArray();
        byte[] arrayOfByte4 = new byte[4];
        int k = arrayOfByte1.length;
        int m = arrayOfByte5.length;
        j = arrayOfByte2.length;
        int n = (4 - k % 4) % 4;
        int i1 = (4 - m % 4) % 4;
        i = (4 - j % 4) % 4;
        byte[] arrayOfByte3 = new byte[k + 12 + n + m + i1 + j + i];
        int i2 = k + n;
        arrayOfByte4[0] = (byte)(i2 & 0xFF);
        arrayOfByte4[1] = (byte)(i2 >> 8 & 0xFF);
        arrayOfByte4[2] = (byte)(i2 >> 16 & 0xFF);
        arrayOfByte4[3] = (byte)(i2 >> 24 & 0xFF);
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, 0, 4);
        System.arraycopy(arrayOfByte1, 0, arrayOfByte3, 4, k);
        k += 4;
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, k, n);
        n = k + n;
        k = m + i1;
        arrayOfByte4[0] = (byte)(k & 0xFF);
        arrayOfByte4[1] = (byte)(k >> 8 & 0xFF);
        arrayOfByte4[2] = (byte)(k >> 16 & 0xFF);
        arrayOfByte4[3] = (byte)(k >> 24 & 0xFF);
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, n, 4);
        n += 4;
        System.arraycopy(arrayOfByte5, 0, arrayOfByte3, n, m);
        m = n + m;
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, m, i1);
        i1 = m + i1;
        m = j + i;
        arrayOfByte4[0] = (byte)(m & 0xFF);
        arrayOfByte4[1] = (byte)(m >> 8 & 0xFF);
        arrayOfByte4[2] = (byte)(m >> 16 & 0xFF);
        arrayOfByte4[3] = (byte)(m >> 24 & 0xFF);
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, i1, 4);
        i1 += 4;
        System.arraycopy(arrayOfByte2, 0, arrayOfByte3, i1, j);
        System.arraycopy(arrayOfByte4, 0, arrayOfByte3, i1 + j, i);
        Log.e("convert", "jpeg");
        i = this.Decode_Vedio_Start_Line_Num + this.Change_Decode_Vedio_Start_Line_Num;
        this.Decode_Vedio_Start_Line_Num = i;
        j = this.Decode_Vedio_Height;
        if (i >= j)
          this.Decode_Vedio_Start_Line_Num = i - j; 
        mat.release();
        return arrayOfByte3;
      } 
      i = this.Center_Operate_Mode;
      if (i == 2) {
        for (i = 0; i < this.Decode_Vedio_Height; i += 3) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i]] = 0;
          arrayOfByte[arrayOfInt[i] + 1] = 0;
          arrayOfByte[arrayOfInt[i] + 2] = 0;
          j = i + 1;
          arrayOfByte[arrayOfInt[j]] = 0;
          arrayOfByte[arrayOfInt[j] + 1] = 0;
          arrayOfByte[arrayOfInt[j] + 2] = 0;
          arrayOfByte[arrayOfInt[i] + 3] = 0;
          arrayOfByte[arrayOfInt[i] + 4] = 0;
          arrayOfByte[arrayOfInt[i] + 5] = 0;
          arrayOfByte[arrayOfInt[j] + 3] = 0;
          arrayOfByte[arrayOfInt[j] + 4] = 0;
          arrayOfByte[arrayOfInt[j] + 5] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 4) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 6] = 0;
          arrayOfByte[arrayOfInt[i] + 7] = 0;
          arrayOfByte[arrayOfInt[i] + 8] = 0;
          j = i + 1;
          arrayOfByte[arrayOfInt[j] + 6] = 0;
          arrayOfByte[arrayOfInt[j] + 7] = 0;
          arrayOfByte[arrayOfInt[j] + 8] = 0;
          j = i + 2;
          arrayOfByte[arrayOfInt[j] + 6] = 0;
          arrayOfByte[arrayOfInt[j] + 7] = 0;
          arrayOfByte[arrayOfInt[j] + 8] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 12) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 9] = 0;
          arrayOfByte[arrayOfInt[i] + 10] = 0;
          arrayOfByte[arrayOfInt[i] + 11] = 0;
          j = i + 2;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
          j = i + 4;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
          j = i + 6;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
          j = i + 7;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
          j = i + 9;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
          j = i + 11;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 2) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 12] = 0;
          arrayOfByte[arrayOfInt[i] + 13] = 0;
          arrayOfByte[arrayOfInt[i] + 14] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 3) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 15] = 0;
          arrayOfByte[arrayOfInt[i] + 16] = 0;
          arrayOfByte[arrayOfInt[i] + 17] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 4) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 18] = 0;
          arrayOfByte[arrayOfInt[i] + 19] = 0;
          arrayOfByte[arrayOfInt[i] + 20] = 0;
          arrayOfByte[arrayOfInt[i] + 21] = 0;
          arrayOfByte[arrayOfInt[i] + 22] = 0;
          arrayOfByte[arrayOfInt[i] + 23] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 8) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 24] = 0;
          arrayOfByte[arrayOfInt[i] + 25] = 0;
          arrayOfByte[arrayOfInt[i] + 26] = 0;
          arrayOfByte[arrayOfInt[i] + 27] = 0;
          arrayOfByte[arrayOfInt[i] + 28] = 0;
          arrayOfByte[arrayOfInt[i] + 29] = 0;
          arrayOfByte[arrayOfInt[i] + 30] = 0;
          arrayOfByte[arrayOfInt[i] + 31] = 0;
          arrayOfByte[arrayOfInt[i] + 32] = 0;
          arrayOfByte[arrayOfInt[i] + 33] = 0;
          arrayOfByte[arrayOfInt[i] + 34] = 0;
          arrayOfByte[arrayOfInt[i] + 35] = 0;
          arrayOfByte[arrayOfInt[i] + 36] = 0;
          arrayOfByte[arrayOfInt[i] + 37] = 0;
          arrayOfByte[arrayOfInt[i] + 38] = 0;
          arrayOfByte[arrayOfInt[i] + 39] = 0;
          arrayOfByte[arrayOfInt[i] + 40] = 0;
          arrayOfByte[arrayOfInt[i] + 41] = 0;
        } 
      } else if (i == 1) {
        for (i = 0; i < this.Decode_Vedio_Height; i += 6) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i]] = 0;
          arrayOfByte[arrayOfInt[i] + 1] = 0;
          arrayOfByte[arrayOfInt[i] + 2] = 0;
          j = i + 1;
          arrayOfByte[arrayOfInt[j]] = 0;
          arrayOfByte[arrayOfInt[j] + 1] = 0;
          arrayOfByte[arrayOfInt[j] + 2] = 0;
          j = i + 2;
          arrayOfByte[arrayOfInt[j]] = 0;
          arrayOfByte[arrayOfInt[j] + 1] = 0;
          arrayOfByte[arrayOfInt[j] + 2] = 0;
          j = i + 3;
          arrayOfByte[arrayOfInt[j]] = 0;
          arrayOfByte[arrayOfInt[j] + 1] = 0;
          arrayOfByte[arrayOfInt[j] + 2] = 0;
          j = i + 4;
          arrayOfByte[arrayOfInt[j]] = 0;
          arrayOfByte[arrayOfInt[j] + 1] = 0;
          arrayOfByte[arrayOfInt[j] + 2] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 4) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 3] = 0;
          arrayOfByte[arrayOfInt[i] + 4] = 0;
          arrayOfByte[arrayOfInt[i] + 5] = 0;
          j = i + 1;
          arrayOfByte[arrayOfInt[j] + 3] = 0;
          arrayOfByte[arrayOfInt[j] + 4] = 0;
          arrayOfByte[arrayOfInt[j] + 5] = 0;
          j = i + 2;
          arrayOfByte[arrayOfInt[j] + 3] = 0;
          arrayOfByte[arrayOfInt[j] + 4] = 0;
          arrayOfByte[arrayOfInt[j] + 5] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 12) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 6] = 0;
          arrayOfByte[arrayOfInt[i] + 7] = 0;
          arrayOfByte[arrayOfInt[i] + 8] = 0;
          int n = i + 2;
          arrayOfByte[arrayOfInt[n] + 6] = 0;
          arrayOfByte[arrayOfInt[n] + 7] = 0;
          arrayOfByte[arrayOfInt[n] + 8] = 0;
          int i2 = i + 4;
          arrayOfByte[arrayOfInt[i2] + 6] = 0;
          arrayOfByte[arrayOfInt[i2] + 7] = 0;
          arrayOfByte[arrayOfInt[i2] + 8] = 0;
          int i1 = i + 6;
          arrayOfByte[arrayOfInt[i1] + 6] = 0;
          arrayOfByte[arrayOfInt[i1] + 7] = 0;
          arrayOfByte[arrayOfInt[i1] + 8] = 0;
          int m = i + 7;
          arrayOfByte[arrayOfInt[m] + 6] = 0;
          arrayOfByte[arrayOfInt[m] + 7] = 0;
          arrayOfByte[arrayOfInt[m] + 8] = 0;
          int k = i + 9;
          arrayOfByte[arrayOfInt[k] + 6] = 0;
          arrayOfByte[arrayOfInt[k] + 7] = 0;
          arrayOfByte[arrayOfInt[k] + 8] = 0;
          j = i + 11;
          arrayOfByte[arrayOfInt[j] + 6] = 0;
          arrayOfByte[arrayOfInt[j] + 7] = 0;
          arrayOfByte[arrayOfInt[j] + 8] = 0;
          arrayOfByte[arrayOfInt[i] + 9] = 0;
          arrayOfByte[arrayOfInt[i] + 10] = 0;
          arrayOfByte[arrayOfInt[i] + 11] = 0;
          arrayOfByte[arrayOfInt[n] + 9] = 0;
          arrayOfByte[arrayOfInt[n] + 10] = 0;
          arrayOfByte[arrayOfInt[n] + 11] = 0;
          arrayOfByte[arrayOfInt[i2] + 9] = 0;
          arrayOfByte[arrayOfInt[i2] + 10] = 0;
          arrayOfByte[arrayOfInt[i2] + 11] = 0;
          arrayOfByte[arrayOfInt[i1] + 9] = 0;
          arrayOfByte[arrayOfInt[i1] + 10] = 0;
          arrayOfByte[arrayOfInt[i1] + 11] = 0;
          arrayOfByte[arrayOfInt[m] + 9] = 0;
          arrayOfByte[arrayOfInt[m] + 10] = 0;
          arrayOfByte[arrayOfInt[m] + 11] = 0;
          arrayOfByte[arrayOfInt[k] + 9] = 0;
          arrayOfByte[arrayOfInt[k] + 10] = 0;
          arrayOfByte[arrayOfInt[k] + 11] = 0;
          arrayOfByte[arrayOfInt[j] + 9] = 0;
          arrayOfByte[arrayOfInt[j] + 10] = 0;
          arrayOfByte[arrayOfInt[j] + 11] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 2) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 12] = 0;
          arrayOfByte[arrayOfInt[i] + 13] = 0;
          arrayOfByte[arrayOfInt[i] + 14] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 3) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 15] = 0;
          arrayOfByte[arrayOfInt[i] + 16] = 0;
          arrayOfByte[arrayOfInt[i] + 17] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 4) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 18] = 0;
          arrayOfByte[arrayOfInt[i] + 19] = 0;
          arrayOfByte[arrayOfInt[i] + 20] = 0;
          arrayOfByte[arrayOfInt[i] + 21] = 0;
          arrayOfByte[arrayOfInt[i] + 22] = 0;
          arrayOfByte[arrayOfInt[i] + 23] = 0;
        } 
        for (i = 0; i < this.Decode_Vedio_Height; i += 8) {
          byte[] arrayOfByte = this.Target_Bin;
          int[] arrayOfInt = this.Lightness_Average_Serial_Array;
          arrayOfByte[arrayOfInt[i] + 24] = 0;
          arrayOfByte[arrayOfInt[i] + 25] = 0;
          arrayOfByte[arrayOfInt[i] + 26] = 0;
          arrayOfByte[arrayOfInt[i] + 27] = 0;
          arrayOfByte[arrayOfInt[i] + 28] = 0;
          arrayOfByte[arrayOfInt[i] + 29] = 0;
          arrayOfByte[arrayOfInt[i] + 30] = 0;
          arrayOfByte[arrayOfInt[i] + 31] = 0;
          arrayOfByte[arrayOfInt[i] + 32] = 0;
          arrayOfByte[arrayOfInt[i] + 33] = 0;
          arrayOfByte[arrayOfInt[i] + 34] = 0;
          arrayOfByte[arrayOfInt[i] + 35] = 0;
          arrayOfByte[arrayOfInt[i] + 36] = 0;
          arrayOfByte[arrayOfInt[i] + 37] = 0;
          arrayOfByte[arrayOfInt[i] + 38] = 0;
          arrayOfByte[arrayOfInt[i] + 39] = 0;
          arrayOfByte[arrayOfInt[i] + 40] = 0;
          arrayOfByte[arrayOfInt[i] + 41] = 0;
        } 
      } else {
        byte b1 = 0;
        j = 0;
        byte b4 = 0;
        byte b2 = 0;
        byte b3 = 0;
        i = 0;
        while (b1 < this.Decode_Vedio_Height) {
          if (j != 0) {
            byte[] arrayOfByte = this.Target_Bin;
            int[] arrayOfInt = this.Lightness_Average_Serial_Array;
            arrayOfByte[arrayOfInt[b1]] = 0;
            arrayOfByte[arrayOfInt[b1] + 1] = 0;
            arrayOfByte[arrayOfInt[b1] + 2] = 0;
            if (b4) {
              arrayOfByte[arrayOfInt[b1] + 3] = 0;
              arrayOfByte[arrayOfInt[b1] + 4] = 0;
              arrayOfByte[arrayOfInt[b1] + 5] = 0;
              if (b2) {
                arrayOfByte[arrayOfInt[b1] + 6] = 0;
                arrayOfByte[arrayOfInt[b1] + 7] = 0;
                arrayOfByte[arrayOfInt[b1] + 8] = 0;
              } 
            } 
          } 
          if (!b3) {
            byte[] arrayOfByte = this.Target_Bin;
            int[] arrayOfInt = this.Lightness_Average_Serial_Array;
            arrayOfByte[arrayOfInt[b1] + 9] = 0;
            arrayOfByte[arrayOfInt[b1] + 10] = 0;
            arrayOfByte[arrayOfInt[b1] + 11] = 0;
            if (i == 0) {
              arrayOfByte[arrayOfInt[b1] + 12] = 0;
              arrayOfByte[arrayOfInt[b1] + 13] = 0;
              arrayOfByte[arrayOfInt[b1] + 14] = 0;
            } 
          } 
          byte b5 = (byte)(i + 1);
          i = b5;
          if (b5 == 12)
            i = 0; 
          b5 = (byte)(j + 1);
          j = b5;
          if (b5 == 24)
            j = 0; 
          b5 = (byte)(b4 + 1);
          b4 = b5;
          if (b5 == 6)
            b4 = 0; 
          b5 = (byte)(b2 + 1);
          b2 = b5;
          if (b5 == 3)
            b2 = 0; 
          b5 = (byte)(b3 + 1);
          b3 = b5;
          if (b5 == 2)
            b3 = 0; 
          b1++;
        } 
      } 
      i = 0;
      byte b = 0;
      while (i < this.Decode_Vedio_Height) {
        if (this.Decode_Vedio_Start_Line_Num == 0)
          this.Decode_Vedio_Start_Line_Num = this.Decode_One_Picture_Target_Size; 
        this.Decode_Vedio_Start_Line_Num -= this.Target_Result_Width;
        j = 0;
        byte b1 = 0;
        while (j < this.Swift_Width_Num) {
          int k = this.Decode_Vedio_Start_Line_Num;
          for (byte b2 = 0; b2 < 3; b2++) {
            int n = (b2 << 3) + b;
            int i1 = n + 1;
            int i2 = n + 2;
            int i3 = n + 3;
            int i4 = n + 4;
            int i5 = n + 5;
            int i6 = n + 6;
            int m = n + 7;
            int i7 = this.Switch_Serial_Initial_Num - b2 + k - b1;
            if (this.Decode_Vedio_Lower_Level_Flag) {
              byte[] arrayOfByte4 = this.Three_Fan_File_Formats_Bin;
              int i9 = this.Switch_Serial_Num_Add_Seven;
              byte[] arrayOfByte3 = this.Target_Bin;
              arrayOfByte4[i7 + i9] = (byte)(arrayOfByte3[n] & 0x1 | (arrayOfByte3[i1] & 0x1) << 1 | (arrayOfByte3[i2] & 0x1) << 2 | (arrayOfByte3[i3] & 0x1) << 3 | (arrayOfByte3[i4] & 0x1) << 4 | (arrayOfByte3[i5] & 0x1) << 5 | (arrayOfByte3[i6] & 0x1) << 6 | (arrayOfByte3[m] & 0x1) << 7);
              arrayOfByte4[i7 + this.Switch_Serial_Num_Add_Six] = (byte)((arrayOfByte3[n] & 0x2) >> 1 | arrayOfByte3[i1] & 0x2 | (arrayOfByte3[i2] & 0x2) << 1 | (arrayOfByte3[i3] & 0x2) << 2 | (arrayOfByte3[i4] & 0x2) << 3 | (arrayOfByte3[i5] & 0x2) << 4 | (arrayOfByte3[i6] & 0x2) << 5 | (arrayOfByte3[m] & 0x2) << 6);
            } 
            byte[] arrayOfByte2 = this.Three_Fan_File_Formats_Bin;
            int i8 = this.Switch_Serial_Num_Add_Five;
            byte[] arrayOfByte1 = this.Target_Bin;
            arrayOfByte2[i7 + i8] = (byte)((arrayOfByte1[n] & 0x4) >> 2 | (arrayOfByte1[i1] & 0x4) >> 1 | arrayOfByte1[i2] & 0x4 | (arrayOfByte1[i3] & 0x4) << 1 | (arrayOfByte1[i4] & 0x4) << 2 | (arrayOfByte1[i5] & 0x4) << 3 | (arrayOfByte1[i6] & 0x4) << 4 | (arrayOfByte1[m] & 0x4) << 5);
            arrayOfByte2[i7 + this.Switch_Serial_Num_Add_Four] = (byte)((arrayOfByte1[n] & 0x8) >> 3 | (arrayOfByte1[i1] & 0x8) >> 2 | (arrayOfByte1[i2] & 0x8) >> 1 | arrayOfByte1[i3] & 0x8 | (arrayOfByte1[i4] & 0x8) << 1 | (arrayOfByte1[i5] & 0x8) << 2 | (arrayOfByte1[i6] & 0x8) << 3 | (arrayOfByte1[m] & 0x8) << 4);
            arrayOfByte2[i7 + this.Switch_Serial_Num_Add_Three] = (byte)((arrayOfByte1[n] & 0x10) >> 4 | (arrayOfByte1[i1] & 0x10) >> 3 | (arrayOfByte1[i2] & 0x10) >> 2 | (arrayOfByte1[i3] & 0x10) >> 1 | arrayOfByte1[i4] & 0x10 | (arrayOfByte1[i5] & 0x10) << 1 | (arrayOfByte1[i6] & 0x10) << 2 | (arrayOfByte1[m] & 0x10) << 3);
            arrayOfByte2[i7 + this.Switch_Serial_Num_Add_Two] = (byte)((arrayOfByte1[n] & 0x20) >> 5 | (arrayOfByte1[i1] & 0x20) >> 4 | (arrayOfByte1[i2] & 0x20) >> 3 | (arrayOfByte1[i3] & 0x20) >> 2 | (arrayOfByte1[i4] & 0x20) >> 1 | arrayOfByte1[i5] & 0x20 | (arrayOfByte1[i6] & 0x20) << 1 | (arrayOfByte1[m] & 0x20) << 2);
            arrayOfByte2[i7 + this.Switch_Serial_Num_Add_One] = (byte)((arrayOfByte1[n] & 0x40) >> 6 | (arrayOfByte1[i1] & 0x40) >> 5 | (arrayOfByte1[i2] & 0x40) >> 4 | (arrayOfByte1[i3] & 0x40) >> 3 | (arrayOfByte1[i4] & 0x40) >> 2 | (arrayOfByte1[i5] & 0x40) >> 1 | arrayOfByte1[i6] & 0x40 | (arrayOfByte1[m] & 0x40) << 1);
            arrayOfByte2[i7] = (byte)((arrayOfByte1[n] & 0x80) >> 7 | (arrayOfByte1[i1] & 0x80) >> 6 | (arrayOfByte1[i2] & 0x80) >> 5 | (arrayOfByte1[i3] & 0x80) >> 4 | (arrayOfByte1[i4] & 0x80) >> 3 | (arrayOfByte1[i5] & 0x80) >> 2 | (arrayOfByte1[i6] & 0x80) >> 1 | arrayOfByte1[m] & 0x80);
          } 
          b += 24;
          b1 += 3;
          j++;
        } 
        i++;
      } 
      this.Decode_Vedio_Start_Line_Num -= this.Change_Decode_Vedio_Start_Line_Num;
      while (true) {
        i = this.Decode_Vedio_Start_Line_Num;
        if (i < 0) {
          this.Decode_Vedio_Start_Line_Num = i + this.Decode_One_Picture_Target_Size;
          continue;
        } 
        break;
      } 
    } 
    if (this.Exchange_Mode == 1) {
      int k = this.Decode_Vedio_Height;
      byte[] arrayOfByte = new byte[4];
      j = 0;
      i = 2;
      while (j < k * 6) {
        byte b;
        for (b = 0; b < 10; b = (byte)(b + 1)) {
          byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
          arrayOfByte[0] = arrayOfByte1[i];
          int n = i + 1;
          arrayOfByte[1] = arrayOfByte1[n];
          int m = i + 2;
          arrayOfByte[2] = arrayOfByte1[m];
          int i1 = i + 3;
          arrayOfByte[3] = arrayOfByte1[i1];
          arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
          byte b5 = arrayOfByte[0];
          byte b6 = arrayOfByte[2];
          byte b3 = arrayOfByte[0];
          byte b4 = arrayOfByte[2];
          byte b2 = arrayOfByte[0];
          byte b1 = arrayOfByte[2];
          arrayOfByte1[n] = (byte)((arrayOfByte[0] & 0x1) << 1 | (b5 & 0x8) << 4 | (b6 & 0x8) << 3 | (b3 & 0x4) << 3 | (b4 & 0x4) << 2 | (b2 & 0x2) << 2 | (b1 & 0x2) << 1 | arrayOfByte[2] & 0x1);
          arrayOfByte1[m] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
          arrayOfByte1[i1] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
          i += 4;
        } 
        i += 2;
        j++;
      } 
    } 
    if (this.Exchange_Mode == 2) {
      int k = this.Decode_Vedio_Height;
      byte[] arrayOfByte = new byte[4];
      j = 0;
      i = 2;
      while (j < k << 3) {
        byte b;
        for (b = 0; b < 10; b = (byte)(b + 1)) {
          byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
          arrayOfByte[0] = arrayOfByte1[i];
          int m = i + 1;
          arrayOfByte[1] = arrayOfByte1[m];
          int n = i + 2;
          arrayOfByte[2] = arrayOfByte1[n];
          int i1 = i + 3;
          arrayOfByte[3] = arrayOfByte1[i1];
          arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
          byte b7 = arrayOfByte[0];
          byte b2 = arrayOfByte[2];
          byte b5 = arrayOfByte[0];
          byte b1 = arrayOfByte[2];
          byte b4 = arrayOfByte[0];
          byte b3 = arrayOfByte[2];
          byte b6 = arrayOfByte[0];
          arrayOfByte1[m] = (byte)(arrayOfByte[2] & 0x1 | (b6 & 0x1) << 1 | (b7 & 0x8) << 4 | (b2 & 0x8) << 3 | (b5 & 0x4) << 3 | (b1 & 0x4) << 2 | (b4 & 0x2) << 2 | (b3 & 0x2) << 1);
          arrayOfByte1[n] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
          arrayOfByte1[i1] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
          i += 4;
        } 
        i += 2;
        j++;
      } 
    } 
    if (this.Exchange_Mode == 3) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 576; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int n = i + 1;
        arrayOfByte[1] = arrayOfByte1[n];
        int m = i + 2;
        arrayOfByte[2] = arrayOfByte1[m];
        int k = i + 3;
        arrayOfByte[3] = arrayOfByte1[k];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b4 = arrayOfByte[0];
        byte b6 = arrayOfByte[2];
        byte b1 = arrayOfByte[0];
        byte b5 = arrayOfByte[2];
        byte b7 = arrayOfByte[0];
        byte b2 = arrayOfByte[2];
        byte b3 = arrayOfByte[0];
        arrayOfByte1[n] = (byte)(arrayOfByte[2] & 0x1 | (b3 & 0x1) << 1 | (b4 & 0x8) << 4 | (b6 & 0x8) << 3 | (b1 & 0x4) << 3 | (b5 & 0x4) << 2 | (b7 & 0x2) << 2 | (b2 & 0x2) << 1);
        arrayOfByte1[m] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[k] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 4) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 288; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int n = i + 1;
        arrayOfByte[1] = arrayOfByte1[n];
        int k = i + 2;
        arrayOfByte[2] = arrayOfByte1[k];
        int m = i + 3;
        arrayOfByte[3] = arrayOfByte1[m];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b7 = arrayOfByte[0];
        byte b3 = arrayOfByte[2];
        byte b2 = arrayOfByte[0];
        byte b4 = arrayOfByte[2];
        byte b5 = arrayOfByte[0];
        byte b1 = arrayOfByte[2];
        byte b6 = arrayOfByte[0];
        arrayOfByte1[n] = (byte)(arrayOfByte[2] & 0x1 | (b6 & 0x1) << 1 | (b7 & 0x8) << 4 | (b3 & 0x8) << 3 | (b2 & 0x4) << 3 | (b4 & 0x4) << 2 | (b5 & 0x2) << 2 | (b1 & 0x2) << 1);
        arrayOfByte1[k] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[m] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 5) {
      int k = this.Decode_Vedio_Height;
      byte[] arrayOfByte = new byte[4];
      j = 0;
      i = 2;
      while (j < k << 3) {
        int m = 0;
        int n = i;
        for (i = m; i < 7; i = (byte)(i + 1)) {
          byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
          arrayOfByte[0] = arrayOfByte1[n];
          int i1 = n + 1;
          arrayOfByte[1] = arrayOfByte1[i1];
          int i2 = n + 2;
          arrayOfByte[2] = arrayOfByte1[i2];
          m = n + 3;
          arrayOfByte[3] = arrayOfByte1[m];
          arrayOfByte1[n] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
          byte b5 = arrayOfByte[0];
          byte b2 = arrayOfByte[2];
          byte b4 = arrayOfByte[0];
          byte b1 = arrayOfByte[2];
          byte b7 = arrayOfByte[0];
          byte b3 = arrayOfByte[2];
          byte b6 = arrayOfByte[0];
          arrayOfByte1[i1] = (byte)(arrayOfByte[2] & 0x1 | (b6 & 0x1) << 1 | (b5 & 0x8) << 4 | (b2 & 0x8) << 3 | (b4 & 0x4) << 3 | (b1 & 0x4) << 2 | (b7 & 0x2) << 2 | (b3 & 0x2) << 1);
          arrayOfByte1[i2] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
          arrayOfByte1[m] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
          n += 4;
        } 
        i = n + 2;
        j++;
      } 
    } 
    if (this.Exchange_Mode == 6) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 672; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int k = i + 1;
        arrayOfByte[1] = arrayOfByte1[k];
        int m = i + 2;
        arrayOfByte[2] = arrayOfByte1[m];
        int n = i + 3;
        arrayOfByte[3] = arrayOfByte1[n];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b5 = arrayOfByte[0];
        byte b2 = arrayOfByte[2];
        byte b4 = arrayOfByte[0];
        byte b3 = arrayOfByte[2];
        byte b1 = arrayOfByte[0];
        byte b7 = arrayOfByte[2];
        byte b6 = arrayOfByte[0];
        arrayOfByte1[k] = (byte)(arrayOfByte[2] & 0x1 | (b6 & 0x1) << 1 | (b5 & 0x8) << 4 | (b2 & 0x8) << 3 | (b4 & 0x4) << 3 | (b3 & 0x4) << 2 | (b1 & 0x2) << 2 | (b7 & 0x2) << 1);
        arrayOfByte1[m] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[n] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 7) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 768; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int m = i + 1;
        arrayOfByte[1] = arrayOfByte1[m];
        int n = i + 2;
        arrayOfByte[2] = arrayOfByte1[n];
        int k = i + 3;
        arrayOfByte[3] = arrayOfByte1[k];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b1 = arrayOfByte[0];
        byte b7 = arrayOfByte[2];
        byte b4 = arrayOfByte[0];
        byte b2 = arrayOfByte[2];
        byte b6 = arrayOfByte[0];
        byte b3 = arrayOfByte[2];
        byte b5 = arrayOfByte[0];
        arrayOfByte1[m] = (byte)(arrayOfByte[2] & 0x1 | (b5 & 0x1) << 1 | (b1 & 0x8) << 4 | (b7 & 0x8) << 3 | (b4 & 0x4) << 3 | (b2 & 0x4) << 2 | (b6 & 0x2) << 2 | (b3 & 0x2) << 1);
        arrayOfByte1[n] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[k] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 8) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 144; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int n = i + 1;
        arrayOfByte[1] = arrayOfByte1[n];
        int k = i + 2;
        arrayOfByte[2] = arrayOfByte1[k];
        int m = i + 3;
        arrayOfByte[3] = arrayOfByte1[m];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b1 = arrayOfByte[0];
        byte b3 = arrayOfByte[2];
        byte b2 = arrayOfByte[0];
        byte b6 = arrayOfByte[2];
        byte b7 = arrayOfByte[0];
        byte b5 = arrayOfByte[2];
        byte b4 = arrayOfByte[0];
        arrayOfByte1[n] = (byte)(arrayOfByte[2] & 0x1 | (b4 & 0x1) << 1 | (b1 & 0x8) << 4 | (b3 & 0x8) << 3 | (b2 & 0x4) << 3 | (b6 & 0x4) << 2 | (b7 & 0x2) << 2 | (b5 & 0x2) << 1);
        arrayOfByte1[k] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[m] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 9) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 192; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int m = i + 1;
        arrayOfByte[1] = arrayOfByte1[m];
        int n = i + 2;
        arrayOfByte[2] = arrayOfByte1[n];
        int k = i + 3;
        arrayOfByte[3] = arrayOfByte1[k];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b3 = arrayOfByte[0];
        byte b7 = arrayOfByte[2];
        byte b4 = arrayOfByte[0];
        byte b1 = arrayOfByte[2];
        byte b5 = arrayOfByte[0];
        byte b6 = arrayOfByte[2];
        byte b2 = arrayOfByte[0];
        arrayOfByte1[m] = (byte)(arrayOfByte[2] & 0x1 | (b2 & 0x1) << 1 | (b3 & 0x8) << 4 | (b7 & 0x8) << 3 | (b4 & 0x4) << 3 | (b1 & 0x4) << 2 | (b5 & 0x2) << 2 | (b6 & 0x2) << 1);
        arrayOfByte1[n] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[k] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    if (this.Exchange_Mode == 10) {
      int k = this.Decode_Vedio_Height;
      byte[] arrayOfByte = new byte[4];
      j = 0;
      i = 2;
      while (j < k << 3) {
        byte b;
        for (b = 0; b < 4; b = (byte)(b + 1)) {
          byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
          arrayOfByte[0] = arrayOfByte1[i];
          int m = i + 1;
          arrayOfByte[1] = arrayOfByte1[m];
          int n = i + 2;
          arrayOfByte[2] = arrayOfByte1[n];
          int i1 = i + 3;
          arrayOfByte[3] = arrayOfByte1[i1];
          arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
          byte b7 = arrayOfByte[0];
          byte b4 = arrayOfByte[2];
          byte b5 = arrayOfByte[0];
          byte b2 = arrayOfByte[2];
          byte b3 = arrayOfByte[0];
          byte b1 = arrayOfByte[2];
          byte b6 = arrayOfByte[0];
          arrayOfByte1[m] = (byte)(arrayOfByte[2] & 0x1 | (b6 & 0x1) << 1 | (b7 & 0x8) << 4 | (b4 & 0x8) << 3 | (b5 & 0x4) << 3 | (b2 & 0x4) << 2 | (b3 & 0x2) << 2 | (b1 & 0x2) << 1);
          arrayOfByte1[n] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
          arrayOfByte1[i1] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
          i += 4;
        } 
        i += 2;
        j++;
      } 
    } 
    if (this.Exchange_Mode == 11) {
      int k = this.Decode_Vedio_Height;
      byte[] arrayOfByte = new byte[4];
      j = 0;
      i = 2;
      while (j < k * 6) {
        byte b;
        for (b = 0; b < 4; b = (byte)(b + 1)) {
          byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
          arrayOfByte[0] = arrayOfByte1[i];
          int n = i + 1;
          arrayOfByte[1] = arrayOfByte1[n];
          int m = i + 2;
          arrayOfByte[2] = arrayOfByte1[m];
          int i1 = i + 3;
          arrayOfByte[3] = arrayOfByte1[i1];
          arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
          byte b5 = arrayOfByte[0];
          byte b1 = arrayOfByte[2];
          byte b7 = arrayOfByte[0];
          byte b6 = arrayOfByte[2];
          byte b2 = arrayOfByte[0];
          byte b4 = arrayOfByte[2];
          byte b3 = arrayOfByte[0];
          arrayOfByte1[n] = (byte)(arrayOfByte[2] & 0x1 | (b3 & 0x1) << 1 | (b5 & 0x8) << 4 | (b1 & 0x8) << 3 | (b7 & 0x4) << 3 | (b6 & 0x4) << 2 | (b2 & 0x2) << 2 | (b4 & 0x2) << 1);
          arrayOfByte1[m] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
          arrayOfByte1[i1] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
          i += 4;
        } 
        i += 2;
        j++;
      } 
    } 
    if (this.Exchange_Mode == 12) {
      byte[] arrayOfByte = new byte[4];
      j = this.Decode_Vedio_Height;
      for (i = 0; i < j * 480; i += 4) {
        byte[] arrayOfByte1 = this.Three_Fan_File_Formats_Bin;
        arrayOfByte[0] = arrayOfByte1[i];
        int k = i + 1;
        arrayOfByte[1] = arrayOfByte1[k];
        int m = i + 2;
        arrayOfByte[2] = arrayOfByte1[m];
        int n = i + 3;
        arrayOfByte[3] = arrayOfByte1[n];
        arrayOfByte1[i] = (byte)(arrayOfByte[0] & 0x80 | (arrayOfByte[2] & 0x80) >> 1 | (arrayOfByte[0] & 0x40) >> 1 | (arrayOfByte[2] & 0x40) >> 2 | (arrayOfByte[0] & 0x20) >> 2 | (arrayOfByte[2] & 0x20) >> 3 | (arrayOfByte[0] & 0x10) >> 3 | (arrayOfByte[2] & 0x10) >> 4);
        byte b1 = arrayOfByte[0];
        byte b2 = arrayOfByte[2];
        byte b6 = arrayOfByte[0];
        byte b7 = arrayOfByte[2];
        byte b4 = arrayOfByte[0];
        byte b3 = arrayOfByte[2];
        byte b5 = arrayOfByte[0];
        arrayOfByte1[k] = (byte)(arrayOfByte[2] & 0x1 | (b1 & 0x8) << 4 | (b2 & 0x8) << 3 | (b6 & 0x4) << 3 | (b7 & 0x4) << 2 | (b4 & 0x2) << 2 | (b3 & 0x2) << 1 | (b5 & 0x1) << 1);
        arrayOfByte1[m] = (byte)(arrayOfByte[1] & 0x80 | (arrayOfByte[3] & 0x80) >> 1 | (arrayOfByte[1] & 0x40) >> 1 | (arrayOfByte[3] & 0x40) >> 2 | (arrayOfByte[1] & 0x20) >> 2 | (arrayOfByte[3] & 0x20) >> 3 | (arrayOfByte[1] & 0x10) >> 3 | (arrayOfByte[3] & 0x10) >> 4);
        arrayOfByte1[n] = (byte)((arrayOfByte[1] & 0x8) << 4 | (arrayOfByte[3] & 0x8) << 3 | (arrayOfByte[1] & 0x4) << 3 | (arrayOfByte[3] & 0x4) << 2 | (arrayOfByte[1] & 0x2) << 2 | (arrayOfByte[3] & 0x2) << 1 | (arrayOfByte[1] & 0x1) << 1 | arrayOfByte[3] & 0x1);
      } 
    } 
    mat.release();
    return this.Three_Fan_File_Formats_Bin;
  }
  
  public byte[] convert2(byte[] paramArrayOfbyte, int paramInt1, int paramInt2, int paramInt3) {

  }
  
  public void prePareForConvert(Rect paramRect, int paramInt1, int paramInt2, int paramInt3, int paramInt4) {
    this.selectedRect = paramRect;
    this.isFirstPhotoInited = false;
    this.initial_rang_flag = true;
    this.startx = paramInt1;
    this.starty = paramInt2;
    this.startwidth = paramInt3;
    this.startHeight = paramInt4;
  }
}
