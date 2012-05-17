#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/resource.h>
//#include <asm/types.h>
#include <linux/videodev2.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <QUdpSocket>

#define WIDTH           960
#define HEIGHT          720
#define WIDTH_ROI       960
#define HEIGHT_ROI      480
#define REGISTER_NO       4
#define BRAKE           200
#define NOPLATE         300
#define PRIO_LEVEL       99
#define NICE_LEVEL      -20
#define ASCI_ESC         27
#define DEV_NAME        "/dev/video1"
#define CLEAR(x) memset (&(x), 0, sizeof (x))


struct buffer {
        void *                  start;
        size_t                  length;
};

struct no_plate {
    int height;
    int width;
    float aspect_ratio;
    float speed;
    float direction;
}; typedef struct no_plate No_plate;


static char *           dev_name        = (char*) DEV_NAME;
static int              fd              = -1;
struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;
struct timeval          tv              = {2, 0};

IplImage* grayscale_1;
IplImage* grayscale_2;

CvMemStorage *storage_1 = 0;
CvMemStorage *storage_2 = 0;
const char* output_1 = "Output_Window_1";
const char* output_2 = "Output_Window_2";
const char* adaptive_1 = "Adaptive_1";
const char* adaptive_2 = "Adaptive_2";
IplImage* thresh_1 = 0;
IplImage* thresh_2 = 0;
IplImage* intern_1 = 0;
IplImage* intern_2 = 0;
pthread_t thread_1;
pthread_t thread_2;



static void stop_capturing                         (void);
static void start_capturing                        (void);
static void uninit_device                          (void);
static void init_device                            (void);
static void close_device                           (void);
static void open_device                            (void);
static void init_mmap                              (void);
static void errno_exit                             (const char * s);
static int xioctl                                  (int fd, int request, void * arg);
static int read_frame                              (unsigned char *pixel_val_1, unsigned char *pixel_val_2);
void convert_yuyv_to_grayscale_not_rescaled        (const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2);
void convert_yuyv_to_grayscale_rescaled            (const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2);
void convert_yuyv_to_rgb                           (const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2);
void camera_properties                             (CvCapture *camera);
int search_no_plate_canditates_thread_1            (IplImage* img);
int search_no_plate_canditates_thread_2            (IplImage* img);
void process_no                                    (No_plate *current, CvPoint *pt);
double angle_detection                             (CvPoint* pt1, CvPoint* pt2, CvPoint* pt0);
void sendSpeedDirection                            (float speed , float direction );



int read_frame_2 (unsigned char *pixel_val_1, unsigned char *pixel_val_2);
void next_frame (unsigned char *pixel_val_1, unsigned char *pixel_val_2);




int main(void)
{
     char c;
     struct timeval start, end, cap;
     long mtime, seconds, useconds;
     pthread_attr_t attr_1;
     pthread_attr_t attr_2;
     struct sched_param priority_level;
     IplImage* p_frame_1;
     IplImage* p_frame_2;

     priority_level.__sched_priority = PRIO_LEVEL;
     int priority = setpriority(PRIO_PROCESS, 0, NICE_LEVEL);
     int scheduler = sched_setscheduler(0, SCHED_FIFO, &priority_level);


    if(priority == 0)
    {
        fprintf (stderr, "PRIORITY_LEVEL: -20\n");
        fprintf (stderr, "PRIORITY_LEVEL: highest priority\n");
        fprintf (stderr, "PRIORITY_LEVEL: process started by super user\n");
    }
    else
    {
        fprintf (stderr, "PRIORITY_LEVEL: 0\n");
        fprintf (stderr, "PRIORITY_LEVEL: normal priority\n");
        fprintf (stderr, "PRIORITY_LEVEL: process started by user\n");
    }
    if(scheduler == 0)
    {
        fprintf (stderr, "SCHEDULER: SCHED_FIFO\n");
        fprintf (stderr, "SCHEDULER: 99 (highest)\n");
    }
    else
    {
        fprintf (stderr, "SCHEDULER: failed\n");
    }

    pthread_attr_init(&attr_1);
    pthread_attr_init(&attr_2);
    pthread_attr_setdetachstate(&attr_1, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setdetachstate(&attr_2, PTHREAD_CREATE_JOINABLE);

    storage_1 = cvCreateMemStorage(0);
    storage_2 = cvCreateMemStorage(0);
    assert(storage_1 != NULL);
    assert(storage_2 != NULL);

    open_device ();
    init_device ();
    start_capturing ();

    thresh_1    = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);
    thresh_2    = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);
    grayscale_1 = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);
    grayscale_2 = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);
    p_frame_1   = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);
    p_frame_2   = cvCreateImage(cvSize(WIDTH_ROI, HEIGHT_ROI), IPL_DEPTH_8U, 1);

/*
    cvNamedWindow(output_1);
    cvNamedWindow(output_2);
    cvNamedWindow(adaptive_1);
    cvNamedWindow(adaptive_2);
*/
    read_frame((unsigned char*)grayscale_1->imageData, (unsigned char*)grayscale_2->imageData);
  //  next_frame((unsigned char*)grayscale_1->imageData, (unsigned char*)grayscale_2->imageData);
    cvSmooth(grayscale_1, p_frame_1, CV_GAUSSIAN, 5, 5);
    cvSmooth(grayscale_2, p_frame_2, CV_GAUSSIAN, 5, 5);

    pthread_create (&thread_1, &attr_1, (void* (*)(void*)) &search_no_plate_canditates_thread_1, (void *) p_frame_1);
    pthread_create (&thread_2, &attr_2, (void* (*)(void*)) &search_no_plate_canditates_thread_2, (void *) p_frame_2);

    while (1)
    {
         gettimeofday (&start, NULL);

         read_frame((unsigned char*)grayscale_1->imageData, (unsigned char*)grayscale_2->imageData);
       //   next_frame((unsigned char*)grayscale_1->imageData, (unsigned char*)grayscale_2->imageData);

         gettimeofday(&cap, NULL);
         seconds = cap.tv_sec - start.tv_sec;
         useconds = cap.tv_usec - start.tv_usec;
         mtime = ((seconds) * 1000 + useconds / 1000.0);
         fprintf (stderr, "Frame grab: %ld miliseconds\n", mtime);

         pthread_join(thread_1, NULL);
         pthread_join(thread_2, NULL);

       cvSmooth(grayscale_1, p_frame_1, CV_GAUSSIAN, 7, 7);
        cvSmooth(grayscale_2, p_frame_2, CV_GAUSSIAN, 7, 7);
     //   cvCopy(grayscale_1, p_frame_1);
      //  cvCopy(grayscale_2, p_frame_2);

         pthread_create (&thread_1, &attr_1, (void* (*)(void*)) &search_no_plate_canditates_thread_1, (void *) p_frame_1);
         pthread_create (&thread_2, &attr_2, (void* (*)(void*)) &search_no_plate_canditates_thread_2, (void *) p_frame_2);

         gettimeofday(&end, NULL);
         seconds = end.tv_sec - start.tv_sec;
         useconds = end.tv_usec - start.tv_usec;
         mtime = ((seconds) * 1000 + useconds / 1000.0);
          fprintf (stderr, "Total time: %ld miliseconds\n", mtime);

         c = cvWaitKey(1);
         if (c == ASCI_ESC)
         {
               break;
         }
    }

/*
    cvDestroyWindow(output_1);
    cvDestroyWindow(output_2);
    cvDestroyWindow(adaptive_1);
    cvDestroyWindow(adaptive_2);
*/
    cvReleaseImage(&thresh_1);
    cvReleaseImage(&thresh_2);
    cvReleaseImage(&p_frame_1);
    cvReleaseImage(&p_frame_2);
    cvReleaseImage(&grayscale_1);
    cvReleaseImage(&grayscale_2);

    cvReleaseMemStorage(&storage_1);
    cvReleaseMemStorage(&storage_2);

    pthread_attr_destroy(&attr_1);
    pthread_attr_destroy(&attr_2);


    //stop_capturing ();
   // uninit_device ();
    //close_device ();

    pthread_exit(NULL);


}


void sendSpeedDirection(float speed , float direction )
{
    QUdpSocket tSocket;
    QByteArray tArray;
    tArray.append( (char)0x06 );
    tArray.append( (char)0x00 );
    tArray.append( (char)0x00 );
    tArray.append( (char)0x00 );

    tArray.append( (char)0x00 );

    //tArray.append( (char)0x00 );
    //tArray.append( (char)0x80 );

    int tSpeed = (( speed/10 + 1 ) * 32000);
    tArray.append( tSpeed / 256 );
    tArray.append( tSpeed % 256 );

    int tDir = ( -direction + 1 ) * 32000;
    tArray.append( tDir / 256 );
    tArray.append( tDir % 256 );

    tArray.append( (char)0x00 );
    tArray.append( (char)0x00 );
    tArray.append( (char)0x00 );
    tArray.append( (char)0x00 );

    tSocket.writeDatagram( tArray , QHostAddress( "192.168.0.20" ) , 13400 );
}


void process_no (No_plate *current, CvPoint *pt)
{
    int x_min, y_min, x_max, y_max, i;

    //alocate min/ max to the first element in array
    x_min = pt->x;
    x_max = pt->x;
    y_min = pt->y;
    y_max = pt->y;

    //search for min and max for both axes
    for (i= 0 ; i<3; i++)
    {
        pt ++;
        if ((pt->x) <= x_min)
        {
            x_min = pt->x;
        }

        if((pt->x) >= x_max)
        {
            x_max = pt->x;
        }
        if ((pt->y) <= y_min)
        {
            y_min = pt->y;
        }

        if((pt->y) >= y_max)
        {
            y_max = pt->y;

        }

    }

    current->height = y_max - y_min;
    current->width =  x_max - x_min;
    current->aspect_ratio = ((float) (current->width) / (current->height)) ;
    current->direction = (float)(x_min + ( current->width / 2)) / WIDTH_ROI;
    current->direction = (current->direction - 0.5)*2;

    if (current->width > BRAKE)
    {
        current->speed = -1;
    }
    else
    {
        current->speed = (float) (NOPLATE- current->width) / NOPLATE;
    }

}





double angle_detection( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return ((dx1*dx2 + dy1*dy2)*(dx1*dx2 + dy1*dy2))/((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2));
}





int search_no_plate_canditates_thread_1( IplImage* process_image)
{
    CvSeq* current_contour;
    int i, j;
    int count = 4;
    CvContourScanner scan_contour;
    double perimeter = 1;


    struct timeval start, end;
    long mtime, seconds, useconds;

   // int p = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
   // printf("Thread_1: cancel type %d\n", p);

    gettimeofday (&start, NULL);




    //++ hard define
    // IplImage* process_image = cvCloneImage( img );
    intern_1 = cvCloneImage(process_image);
    No_plate current;
    CvPoint pt[4];
    CvPoint *rect = pt;
    CvSeq* result;
    double s, t;
   // int thr =0;

    //printf("Thread 1\n");

    //cvSmooth(process_image, process_image, CV_GAUSSIAN, 5, 5);

    for (j=0; j<1; j++)
    {
        if(j==1)
        {
           cvCanny( process_image, thresh_1, 0, 50, 3);
           cvDilate( thresh_1, thresh_1, 0, 1 );
       ///    cvShowImage(adaptive_1, thresh_1);
        }


        if(j==0)
        {
            cvAdaptiveThreshold(process_image, thresh_1, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 7, -2);
            //cvErode( thresh_1, thresh_1, 0, 1 );
            cvDilate(thresh_1, thresh_1, 0, 1);
        ///    cvShowImage(adaptive_1, thresh_1);
        }

        if(j==2)
         {
             cvAdaptiveThreshold(process_image, thresh_1, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 7, 5);
             cvErode( thresh_1, thresh_1, 0, 1 );
        ///     cvShowImage(adaptive_2, thresh_1);

         }

       scan_contour = cvStartFindContours(thresh_1, storage_1, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0) );

        while( (current_contour = cvFindNextContour( scan_contour)) != NULL )
        {

                result = cvApproxPoly( current_contour, sizeof(CvContour), storage_1, CV_POLY_APPROX_DP, cvContourPerimeter(current_contour)*0.07, 0 ); //should eliminate cvContourPerimeters  | change here

                if( result->total == 4 && cvContourArea(result,CV_WHOLE_SEQ,0) > 200 && cvCheckContourConvexity(result) )
                {
                    s = 0;

                    for( i = 0; i < 5; i++ )
                    {
                        if( i >= 2 )
                        {
                            t = angle_detection((CvPoint*)cvGetSeqElem( result, i ),(CvPoint*)cvGetSeqElem( result, i-2 ), (CvPoint*)cvGetSeqElem( result, i-1 ));
                            s = s > t ? s : t;
                         }
                    }

                    if( s < 0.0225 )
                    {
                        //cvSeqPush( squares,(CvPoint*daptive_)cvGetSeqElem( result, i ));
                        pt[0] = *(CvPoint*)cvGetSeqElem( result, 0 );
                        pt[1] = *(CvPoint*)cvGetSeqElem( result, 1 );
                        pt[2] = *(CvPoint*)cvGetSeqElem( result, 2 );
                        pt[3] = *(CvPoint*)cvGetSeqElem( result, 3 );

                        process_no (&current, pt);

                        if((current.aspect_ratio > 2.5) && (current.aspect_ratio < 6))

                        {

                              cvPolyLine( intern_1, &rect, &count, 1, 1,  CV_RGB(255,255,255), 3, CV_AA, 0 );
                             // sendSpeedDirection(current.speed , current.direction );
                               printf("NO plate found \n");
                               /*
                               printf("perimeter = %f\n", perimeter);
                               printf("Aspect ratio: %f\n", current.aspect_ratio);
                               printf("Direction: %f\ndaptive_", current.direction);
                               printf("Speed: %f\n", current.speed);
                               printf("Width: %d\n\n", current.width);
                               */
                   ///             cvShowImage(output_1, intern_1);
                                current_contour = cvEndFindContours( &scan_contour );
                                cvClearMemStorage(storage_1 );
                                //cvReleaseImage( &process_image);
                                cvReleaseImage( &intern_1);


                                gettimeofday(&end, NULL);
                                seconds = end.tv_sec - start.tv_sec;
                                useconds = end.tv_usec - start.tv_usec;
                                mtime = ((seconds) * 1000 + useconds / 1000.0);
                                printf("Thread_1: Elapsed time: %ld miliseconds EXIT 0\n", mtime);

                                //return 0;
                              //  thr = pthread_cancel(thread_2);
                              //  printf("THR_CANCE_THREAD2 = %d\n", thr);
                                pthread_exit(NULL);
                        }
                        else
                        {
                               cvPolyLine( intern_1, &rect, &count, 1, 1,  CV_RGB(0,0,0), 3, CV_AA, 0 );
                               //printf("NOT FOUND\n");
                               /*
                               printf("Aspect ratio: %f\n", current.aspect_ratio);
                               printf("Direction: %f\n", current.direction);
                               printf("Speed: %f\n", current.speed);
                               printf("Width: %d\n\n", current.width);
                               */
                    ///           cvShowImage(output_1, intern_1);

                        }


                    }
               }



            }
      }

  ///  cvShowImage(output_1, intern_1);
    current_contour = cvEndFindContours( &scan_contour );
    cvClearMemStorage(storage_1 );
   // cvReleaseImage( &process_image);
    cvReleaseImage( &intern_1);

    gettimeofday(&end, NULL);
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    mtime = ((seconds) * 1000 + useconds / 1000.0);
    printf("Threrch_no_plate_canditates_thread_1( Iad_1: Elapsed time: %ld miliseconds EXIT 1\n", mtime);

    //return 1;
    pthread_exit(NULL);

}


int search_no_plate_canditates_thread_2( IplImage* process_image)
{
    CvSeq* current_contour;
    int i, j;
    int count = 4;
    CvContourScanner scan_contour;
    double perimeter = 2;

    struct timeval start, end;
    long mtime, seconds, useconds;
    gettimeofday (&start, NULL);

    intern_2 = cvCloneImage(process_image);
    No_plate current;
    CvPoint pt[4];
    CvPoint *rect = pt;
    CvSeq* result;
    double s, t;
  //  int thr = 0;

   // printf("Thread 2\n");
   // cvSmooth(process_image, process_image, CV_GAUSSIAN, 5, 5);

    for (j=1; j<2; j++)
    {
        if(j==2)
        {
           cvCanny( process_image, thresh_2, 0, 50, 3);
           cvDilate( thresh_2, thresh_2, 0, 1 );
      ///    cvShowImage(adaptive_2, thresh_2);
        }


        if(j==0)
        {
            cvAdaptiveThreshold(process_image, thresh_2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 15, -5);
            cvErode( thresh_2, thresh_2, 0, 1 );
    ///       cvShowImage(adaptive_1, thresh_2);
        }

        if(j==1)
         {
             cvAdaptiveThreshold(process_image, thresh_2, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 7, 0);
            cvDilate( thresh_2, thresh_2, 0, 1 );
      ///      cvShowImage(adaptive_2, thresh_2);

         }


       scan_contour = cvStartFindContours(thresh_2, storage_2, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0) );

        while( (current_contour = cvFindNextContour( scan_contour)) != NULL )
        {

                result = cvApproxPoly( current_contour, sizeof(CvContour), storage_2, CV_POLY_APPROX_DP, cvContourPerimeter(current_contour)*0.07, 0 ); //should eliminate cvContourPerimeters  | change here

                if( result->total == 4 && cvContourArea(result,CV_WHOLE_SEQ,0) > 100 && cvCheckContourConvexity(result) )
                {
                    s = 0;

                    for( i = 0; i < 5; i++ )
                    {
                        if( i >= 2 )
                        {
                            t = angle_detection((CvPoint*)cvGetSeqElem( result, i ),(CvPoint*)cvGetSeqElem( result, i-2 ), (CvPoint*)cvGetSeqElem( result, i-1 ));
                            s = s > t ? s : t;
                         }
                    }

                    if( s < 0.0225 )
                    {
                        //cvSeqPush( squares,(CvPoint*)cvGetSeqElem( result, i ));
                        pt[0] = *(CvPoint*)cvGetSeqElem( result, 0 );
                        pt[1] = *(CvPoint*)cvGetSeqElem( result, 1 );
                        pt[2] = *(CvPoint*)cvGetSeqElem( result, 2 );
                        pt[3] = *(CvPoint*)cvGetSeqElem( result, 3 );

                        process_no (&current, pt);

                        if((current.aspect_ratio > 2.5) && (current.aspect_ratio < 6))

                        {
                                cvPolyLine( intern_2, &rect, &count, 1, 1,  CV_RGB(255,255,255), 3, CV_AA, 0 );
                          //      sendSpeedDirection(current.speed , current.direction );
                               printf("NO plate found \n");
                               /*
                               printf("perimeter = %f\n", perimeter);
                               printf("Aspect ratio: %f\n", current.aspect_ratio);
                               printf("Direction: %f\n", current.direction);
                               printf("Speed: %f\n", current.speed);
                               printf("Width: %d\n\n", current.width);
                               */
                    ///            cvShowImage(output_2, intern_2);
                                current_contour = cvEndFindContours( &scan_contour );
                                cvClearMemStorage(storage_2 );
                                //cvReleaseImage( &process_image);
                                cvReleaseImage( &intern_2);

                                //return 0;
                                //thr = pthread_cancel(thread_1);
                                // printf("THR_CANCE_THREAD1 = %d\n", thr);

                                gettimeofday(&end, NULL);
                                seconds = end.tv_sec - start.tv_sec;
                                useconds = end.tv_usec - start.tv_usec;
                                mtime = ((seconds) * 1000 + useconds / 1000.0);
                                printf("Thread_2: Elapsed time: %ld miliseconds EXIT 0\n", mtime);

                                pthread_exit(NULL);
                        }
                        else
                        {
                              cvPolyLine( intern_2, &rect, &count, 1, 1,  CV_RGB(0,0,0), 3, CV_AA, 0 );
                              // printf("NOT FOUND\n");
                               /*
                               printf("Aspect ratio: %f\n", current.aspect_ratio);
                               printf("Direction: %f\n", current.direction);
                               printf("Speed: %f\n", current.speed);
                           return 0;    printf("Width: %d\n\n", current.width);
                               */
                   ///            cvShowImage(output_2, intern_2);

                        }


                    }
               }



            }
      }

///   cvShowImage(output_2, intern_2);
   current_contour = cvEndFindContours( &scan_contour );
   cvClearMemStorage(storage_2 );
   //cvReleaseImage( &process_image);
   cvReleaseImage( &intern_2);

   //return 1;

   gettimeofday(&end, NULL);
   seconds = end.tv_sec - start.tv_sec;
   useconds = end.tv_usec - start.tv_usec;
   mtime = ((seconds) * 1000 + useconds / 1000.0);
   printf("Thread_2: Elapsed time: %ld miliseconds EXIT 1\n", mtime);

   pthread_exit(NULL);

}









void convert_yuyv_to_grayscale_not_rescaled(const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
    register int y, x, i;
    register int y1, y2, y3, y4, templ;
    register int offset = HEIGHT - HEIGHT_ROI;


    for ( y = offset ; y < HEIGHT ; y++ )
    {
        for (x = 0 ; x <  WIDTH ; x+= 4 )
        {
            templ = (y * WIDTH + x)<<1;
            y1 = ((unsigned char*)p)[ templ + 0 ];
            y2 = ((unsigned char*)p)[ templ + 2 ];
            y3 = ((unsigned char*)p)[ templ + 4 ];
            y4 = ((unsigned char*)p)[ templ + 6 ];

            i = (WIDTH * (y-offset) + x);

            pixel_val_1[i]  = pixel_val_2[i]   = y1;
            pixel_val_1[i+1]= pixel_val_2[i+1] = y2;
            pixel_val_1[i+2]= pixel_val_2[i+2] = y3;
            pixel_val_1[i+3]= pixel_val_2[i+3] = y4;


        }
    }
}




/*get image from buffer and copy to 2 IplImages */
void convert_yuyv_to_grayscale_rescaled(const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
    register int y, x, i;
    register int y1, y2, y3, y4, templ;
    register int offset = HEIGHT - HEIGHT_ROI;


    for ( y = offset ; y < HEIGHT ; y++ )
    {
        for (x = 0 ; x <  WIDTH ; x+= 4 )
        {
            templ = (y * WIDTH + x)<<1;
            y1 = ((((unsigned char*)p)[ templ + 0 ] - 16)*298)>>8;
            y2 = ((((unsigned char*)p)[ templ + 2 ] - 16)*298)>>8;
            y3 = ((((unsigned char*)p)[ templ + 4 ] - 16)*298)>>8;
            y4 = ((((unsigned char*)p)[ templ + 6 ] - 16)*298)>>8;

            if(y1<0) y1 = 0;
            if(y2<0) y2 = 0;
            if(y3<0) y3 = 0;
            if(y4<0) y4 = 0;

            if(y1>255) y1 = 255;
            if(y2>255) y2 = 255;
            if(y3>255) y3 = 255;
            if(y4>255) y4 = 255;


            i = (WIDTH * (y-offset) + x);

            pixel_val_1[i]  = pixel_val_2[i]   = y1;
            pixel_val_1[i+1]= pixel_val_2[i+1] = y2;
            pixel_val_1[i+2]= pixel_val_2[i+2] = y3;
            pixel_val_1[i+3]= pixel_val_2[i+3] = y4;


        }
    }
}




void convert_yuyv_to_rgb (const void *p, unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
    register int y, x, i;
    register int y1, y2, u, v, b1, g1, r1, b2, g2, r2;
    register int offset = HEIGHT - HEIGHT_ROI;
    register int tempy1, tempy2, templ;

    for ( y = offset ; y < HEIGHT ; y++ )
    {
        for (x = 0 ; x <WIDTH; x+= 2 )
        {
            templ =  (y * WIDTH + x)<<1;

            y1 = ((unsigned char*)p)[ templ + 0 ];
            y2 = ((unsigned char*)p)[ templ + 2 ];
            u  = ((unsigned char*)p)[ templ + 1 ];
            v  = ((unsigned char*)p)[ templ + 3 ];

            tempy1 = 298*(y1 - 16);
            tempy2 = 298*(y2 - 16);

            b1 = (tempy1 + 516*(u - 128))>>8;
            g1 = (tempy1 - 208*(v - 128) - 100*(u - 128))>>8;
            r1 = (tempy1 + 409*(v - 128))>>8;

            b2 = (tempy2 + 516*(u - 128))>>8;
            g2 = (tempy2 - 208*(v - 128) - 100*(u - 128))>>8;
            r2 = (tempy2 + 409*(v - 128))>>8;

            if (r1 < 0) r1 = 0;
            if (r2 < 0) r2 = 0;
            if (g1 < 0) g1 = 0;
            if (g2 < 0) g2 = 0;
            if (b1 < 0) b1 = 0;
            if (b2 < 0) b2 = 0;

            if (r1 > 255) r1 = 255;
            if (r2 > 255) r2 = 255;
            if (g1 > 255) g1 = 255;
            if (g2 > 255) g2 = 255;
            if (b1 > 255) b2 = 255;
            if (b2 > 255) b2 = 255;

            i = (WIDTH * (y-offset) + x);

            pixel_val_1[3*i]  = pixel_val_2[3*i]  = b1;
            pixel_val_1[3*i+1]= pixel_val_2[3*i]  = g1;
            pixel_val_1[3*i+2]= pixel_val_2[3*i]  = r1;
            pixel_val_1[3*i+3]= pixel_val_2[3*i]  = b2;
            pixel_val_1[3*i+4]= pixel_val_2[3*i]  = g2;
            pixel_val_1[3*i+5]= pixel_val_2[3*i]  = r2;


        }
    }
}




static void errno_exit  (const char * s)
{
        fprintf (stderr, "%s error %d, %s\n", s, errno, strerror (errno));
        exit (EXIT_FAILURE);
}




static int xioctl  (int fd, int request, void * arg)
{
        int r;
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
        return r;
}




static int read_frame (unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
        fd_set fds;
        struct v4l2_buffer buf;
        int r;
        CLEAR (buf);

        FD_ZERO (&fds);
        FD_SET (fd, &fds);


        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        r = select (fd + 1, &fds, NULL, NULL, &tv);

        if (0 == r) {
                fprintf (stderr, "select timeout\n");
            //    exit (EXIT_FAILURE);
        }

       if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf))
       {
           return 0;
       }

       assert (buf.index < n_buffers);
       convert_yuyv_to_grayscale_rescaled(buffers[buf.index].start, pixel_val_1, pixel_val_2 );
      if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
       errno_exit ("VIDIOC_QBUF");

       return 1;
}


void next_frame (unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_SET (fd, &fds);

        /* Timeout.
        tv.tv_sec = 2;
        tv.tv_usec = 0;*/

        r = select (fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;

            fprintf (stderr, "select timeout\n");
            exit (EXIT_FAILURE);
        }

        if (0 == r) {
            fprintf (stderr, "select timeout\n");
            exit (EXIT_FAILURE);
        }

        if ( read_frame_2 (pixel_val_1, pixel_val_2) )
            break;

        /* EAGAIN - continue select loop. */
    }


}

int read_frame_2 (unsigned char *pixel_val_1, unsigned char *pixel_val_2)
{
    struct v4l2_buffer buf;
    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:
            return 0;

        case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

        default:
        {
            fprintf (stderr, "VIDIOC_DQBUF");
            exit (EXIT_FAILURE);
        }

        }
    }

    assert (buf.index < n_buffers);

    convert_yuyv_to_grayscale_rescaled(buffers[buf.index].start, pixel_val_1, pixel_val_2 );

    if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
    {
        fprintf (stderr, "VIDIOC_DQBUF");
        exit (EXIT_FAILURE);
    }

    return 1;
}




static void stop_capturing   (void)
{
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        printf("Stop_capturing\n");

         if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
               errno_exit ("VIDIOC_STREAMOFF");


}




static void start_capturing (void)
{
        unsigned int i;
        enum v4l2_buf_type type;


        for (i = 0; i < n_buffers; ++i)
        {
             struct v4l2_buffer buf;
             CLEAR (buf);
             buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
             buf.memory      = V4L2_MEMORY_MMAP;
             buf.index       = i;

             if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
             errno_exit ("VIDIOC_QBUF");
        }

       type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

       if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
       errno_exit ("VIDIOC_STREAMON");
}





static void uninit_device (void)
{
        unsigned int i;

        for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap (buffers[i].start, buffers[i].length))
        errno_exit ("munmap");
        free (buffers);
}




static void init_mmap (void)
{
        struct v4l2_requestbuffers req;

        CLEAR (req);

        req.count               = REGISTER_NO;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno)
                {
                    fprintf (stderr, "%s does not support ""memory mapping\n", dev_name);
                    exit (EXIT_FAILURE);
                }
                else
                {
                    errno_exit ("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2)
        {
           fprintf (stderr, "Insufficient buffer memory on %s\n",dev_name);
           exit (EXIT_FAILURE);
        }

        buffers = (buffer*) calloc (req.count, sizeof (*buffers));

        if (!buffers)
        {
            fprintf (stderr, "Out of memory\n");
            exit (EXIT_FAILURE);
        }

        fprintf (stderr, "%d Buffers allocated on %s\n",req.count, dev_name);

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
                errno_exit ("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap (NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit ("mmap");
        }
}






static void init_device (void)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;

        unsigned int min;


        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap))
        {
             if (EINVAL == errno)
             {
                  fprintf (stderr, "%s is no V4L2 device\n", dev_name);
                  exit (EXIT_FAILURE);
             }
             else
             {
                   errno_exit ("VIDIOC_QUERYCAP");
             }
        }


        if ((!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) || (!(cap.capabilities & V4L2_CAP_STREAMING)))
        {
                fprintf (stderr, "%s no capture device or no streaming\n", dev_name);
                exit (EXIT_FAILURE);
        }


        /* Select video input, video standard and tune here. */


        CLEAR (cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }


        CLEAR (fmt);


        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = WIDTH;
        fmt.fmt.pix.height      = HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
              errno_exit ("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */

/*
        v4l2_ext_controls controls;
        //memset(&controls, 0 , sizeof(controls));
        CLEAR(controls);
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count = 3;

        v4l2_ext_control controls_array[3];
        controls.controls = controls_array;

        controls_array[0].id = V4L2_CID_EXPOSURE_AUTO;
        controls_array[0].value = V4L2_EXPOSURE_MANUAL;

        controls_array[1].id = V4L2_CID_EXPOSURE_ABSOLUTE;
        controls_array[1].value = 0;

        controls_array[2].id = V4L2_CID_GAIN;
        controls_array[2].value = 6;


        if (-1 == xioctl (fd, VIDIOC_S_EXT_CTRLS , &controls))
        errno_exit ("Error init device at VIDIOC_S_CTRL");

*//* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        init_mmap ();

}




static void close_device (void)
{
        if (-1 == close (fd))
                errno_exit ("close");

        fd = -1;
}




static void open_device (void)
{
        struct stat st;

        if (-1 == stat (dev_name, &st))
        {
            fprintf (stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror (errno));
            exit (EXIT_FAILURE);
        }

        if (!S_ISCHR (st.st_mode)) {
                fprintf (stderr, "%s is no device\n", dev_name);
                exit (EXIT_FAILURE);
        }

        fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf (stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }
        fprintf (stderr, "Device: '%s' opened\n", dev_name);

}


void camera_properties (CvCapture *camera)
{
    int w, h;
    w = (int) cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH);
    h = (int) cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT);
    printf("Camera properties: width %d X height %d\n", w, h);
}
