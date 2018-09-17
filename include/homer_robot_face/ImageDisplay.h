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

#ifndef TALKING_HEAD_INCLUDE_IMAGEDISPLAY_H_
#define TALKING_HEAD_INCLUDE_IMAGEDISPLAY_H_

#include <string>
#include <queue>

#include <QWidget>
#include <QTimer>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

//message callbacks
#include <homer_robot_face/DisplayImageFile.h>    //for file from harddisk
#include <homer_robot_face/DisplayImage.h>    //for stream data

/**
 * @class  ImageDisplay
 * @brief  Displays an image or image stream for visualization in robot_face framework (complete rework of ImageDisplay from R18)
 * @author Matthias von Steimker (R22)
 */
class ImageDisplay : public QWidget
{
  Q_OBJECT

  public:
    /** Constructor */
    ImageDisplay(const int text_rotation, const unsigned int max_width, const unsigned int max_height, QWidget *parent );
    /** Destructor */
    ~ImageDisplay();
    /** @brief callback for sensorMsgs::Images encapsulated in an ImageDisplay msg*/
    void callbackImageDisplay( const homer_robot_face::DisplayImage::ConstPtr& image_msg );
    /** @brief callback for images from harddisk*/
    void callbackImageFileDisplay(const homer_robot_face::DisplayImageFile::ConstPtr& image_msg );

public slots:
    /** @brief hide the current image and stop the reset timer*/
    void clearImage();
    /** @brief show the current image for time of milliseconds. If there is no image to show, the window would be black*/
    void showImage(const unsigned int milliseconds);

signals:
    void imageProcessed();
    void newImageToShow(const unsigned int milliseconds);

protected:
        void paintEvent(QPaintEvent*);

private:
        /** @brief queue to buffer images (necessary if the frequency of incoming images is higher than the display frequency)*/
        std::queue< QImage* > image_queue_;
        /** @brief timer to specify the duration of painting the current image*/
        QTimer* reset_timer_;   //no smart pointer when using parenting of Qt

        const unsigned int max_image_width_;    //in px
        const unsigned int max_image_height_;   //in px

        int window_rotation_;     //in degrees and only 0, 90, 180 and 270 are allowed

        /** @brief pop the front element in queue and call pointers objects destructor*/
        void pop_image_from_queue();
        /** @brief rotate and draw the current image. Rotation is respectively to window_rotation_*/
        void rotateAndDrawImage(QPainter *painter , const QImage* toDraw);
        /** @brief scale image respectively to max_image_width_ or max_image_height_ and add it to the image queue*/
        void scaleAndAddImageToQueue(const QImage &image );
        /** @brief convert a cv::Mat to a QImage. Supported encodings are CV_8UC4, CV_8UC3 and CV_8UC1*/
        QImage cvMatToQImage( const cv::Mat &inMat );
        /** @brief actual executive process of image data for painting in the window*/
        void processImageData(const homer_robot_face::DisplayImage::ConstPtr& image_msg);
};

#endif // TALKING_HEAD_INCLUDE_IMAGEDISPLAY_H_
