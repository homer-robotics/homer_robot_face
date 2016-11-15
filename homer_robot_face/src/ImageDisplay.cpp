/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2014 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#include <homer_robot_face/ImageDisplay.h>

#include <QPainter>
#include <cv.h>

#define MAX_QUEUE_SIZE 5 //maximum number of images to be buffered

ImageDisplay::ImageDisplay(const int window_rotation, const unsigned int max_width, const unsigned int max_height, QWidget *parent):
    QWidget( parent ),
    max_image_width_( max_width ),
    max_image_height_( max_height ),
    window_rotation_(window_rotation),
    image_queue_(),
    reset_timer_(new QTimer( this ))  // create internal timer
{
    this->setVisible( false );
    reset_timer_->setSingleShot( true );

    connect( reset_timer_, SIGNAL ( timeout() ), SLOT ( clearImage() ) );
    connect( this, SIGNAL ( newImageToShow(unsigned int) ), SLOT ( showImage(unsigned int) ) );
}

ImageDisplay::~ImageDisplay()
{
    while(image_queue_.size() > 0)
    {
        pop_image_from_queue();
    }
    //note: reset_timer will be deleted by parent
}

void ImageDisplay::pop_image_from_queue()
{
    QImage* image = image_queue_.front();
    image_queue_.pop();
    if(image) delete image;
}

void ImageDisplay::showImage(const unsigned int milliseconds)
{
//    ROS_INFO("show it to me");
    this->show();
    update();
    reset_timer_->start( milliseconds);
}

void ImageDisplay::clearImage()
{
//    ROS_INFO("go sleep again");
    this->hide();
    pop_image_from_queue();
}

void ImageDisplay::paintEvent(QPaintEvent*)
{
    QPainter qpainter(this);

    switch(image_queue_.size())
    {
    case 0:
        //no image to paint
        return;
    case 1:
        //if there is only one element in the queue we do not remove it here, because we have to wait until the timer fires;
        rotateAndDrawImage( &qpainter, image_queue_.front() );
        update();
        break;
    default:
        //more than one image in the queue, paint the image at the front of the queue and pop it then
        rotateAndDrawImage( &qpainter, image_queue_.front() );
        update();
        pop_image_from_queue();
        break;
    }
}

void ImageDisplay::rotateAndDrawImage(QPainter *painter, const QImage *toDraw )
{
    painter->save();
    QMatrix matrix;

    switch( window_rotation_ )
    {
    case 0:
        matrix.translate(0, 0);
        matrix.rotate(window_rotation_);
        painter->setMatrix(matrix);
        painter->drawImage(0, 0, *toDraw);
        break;
    case 90:
        matrix.translate(width(), 0);
        matrix.rotate(window_rotation_);
        painter->setMatrix(matrix);
        painter->drawImage(0, 0, *toDraw);
        break;
    case 180:
        matrix.translate(width(), height());
        matrix.rotate(window_rotation_);
        painter->setMatrix(matrix);
        painter->drawImage(0, 0, *toDraw);
        break;
    case 270:
        matrix.translate(0, height());
        matrix.rotate(window_rotation_);
        painter->setMatrix(matrix);
        painter->drawImage(0, 0, *toDraw);
        break;
    default:
        ROS_WARN("ImageDisplay::rotateAndDrawImage() - text_rotation type not 0, 80, 180 or 270");
        break;
    }
    painter->restore();
}

inline void ImageDisplay::scaleAndAddImageToQueue( const QImage& image )
{
    //we don't want to overfill our queue
    if( image_queue_.size() > MAX_QUEUE_SIZE ){
        return;
    }

    //scale image to the greater value of height or width to fit in widget
    QImage* image_ptr;
    if(image.height() > image.width())
    {
        image_ptr = new QImage(image.scaledToHeight(max_image_height_));
    } else {
        image_ptr = new QImage(image.scaledToWidth(max_image_width_));
    }

    //forcing a deep copy
    if(!image_ptr->isDetached())
    {
        image_ptr->detach();
    }

    image_queue_.push(image_ptr);
}

///Convert function from http://asmaloney.com/2013/11/code/converting-between-cvmat-and-qimage-or-qpixmap/
inline QImage ImageDisplay::cvMatToQImage( const cv::Mat &inMat )
{
    switch ( inMat.type() )
    {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );
        return image;
    }
    // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );
        return image.rgbSwapped();
    }
    // 8-bit, 1 channel
    case CV_8UC1:
    {
        static QVector<QRgb>  sColorTable;

        // only create our color table once
        if ( sColorTable.isEmpty() )
        {
            for ( int i = 0; i < 256; ++i )
                sColorTable.push_back( qRgb( i, i, i ) );
        }

        QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );
        image.setColorTable( sColorTable );
        return image;
    }
    default:
        ROS_WARN_STREAM("ImageDisplay::cvMatToQImage() - cv::Mat image type not handled in switch: " << inMat.type());
        break;
    }
    return QImage();
}


void ImageDisplay::processImageData(const homer_robot_face::DisplayImage::ConstPtr& image_msg)
{
    //Get image into OpenCV format..
    cv_bridge::CvImagePtr sensor_img_cv;
    try
    {
        sensor_img_cv = cv_bridge::toCvCopy(image_msg->Image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //scale image to fit in (max_image_width_, max_image_height_)
    cv::Mat resized_img_mat;

    unsigned int width;
    unsigned int height;

//    if(image.height() > image.width())
//    {
//        image_ptr = new QImage(image.scaledToHeight(max_image_height_));
//    } else {
//        image_ptr = new QImage(image.scaledToWidth(max_image_width_));
//    }

    //cols == width
    //rows == height


  //  if(sensor_img_cv->image.cols > sensor_img_cv->image.rows)
  //  {
  //      cv::resize( sensor_img_cv->image, resized_img_mat, cv::Size(sensor_img_cv->image.cols, max_image_height_),0,0, cv::INTER_AREA);
  //  }


    if(sensor_img_cv->image.cols > max_image_width_)
    {
        width = max_image_width_;
    } else {
        width = sensor_img_cv->image.cols;
    }
   // height = max_image_height_;
    //CV_INTER_AREA looks best for shrinked images
    cv::resize( sensor_img_cv->image, resized_img_mat, cv::Size(width, max_image_height_),0,0, cv::INTER_AREA);

    // and from OpenCV to QImage
    QImage* image_converted = new QImage( cvMatToQImage(resized_img_mat) );
    image_queue_.push(image_converted);
}

void ImageDisplay::callbackImageDisplay( const homer_robot_face::DisplayImage::ConstPtr& image_msg)
{
//    ROS_INFO_STREAM("Received ImageDisplay msg, draw image for " << image_msg->time << " seconds.");

    processImageData(image_msg);
    emit newImageToShow( image_msg->time * 1000 ); //convert milliseconds to seconds
}

void ImageDisplay::callbackImageFileDisplay( const homer_robot_face::DisplayImageFile::ConstPtr& image_msg )
{
    ROS_INFO_STREAM("Received ImageFileDisplay msg, draw image '" << image_msg->filename << "' for " << image_msg->time << " seconds.");

    //load a single image
    QString image_file = (image_msg->filename).c_str();
    QImage image_loaded( image_file );

    scaleAndAddImageToQueue(image_loaded);
    emit newImageToShow( image_msg->time * 1000 );  //convert milliseconds to seconds
}

#undef MAX_QUEUE_SIZE
