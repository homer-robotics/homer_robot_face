/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
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

#ifndef TALKING_HEAD_INCLUDE_MAINWINDOW_H_
#define TALKING_HEAD_INCLUDE_MAINWINDOW_H_

#include <QtWidgets>

#include <ros/ros.h>

#include <homer_robot_face/TalkingHead.h>
#include <homer_robot_face/TextOutDisplay.h>
#include <homer_robot_face/FestivalGenerator.h>
#include <homer_robot_face/ImageDisplay.h>

/**
 * @class  MainWindow
 * @brief  Controls and displays Widgets
 * @author Julian Giesen (R16)(R18)
 */

class MainWindow : public QWidget
{
    Q_OBJECT

public:

    /** Constructor */
    explicit MainWindow( QWidget* parent = 0 );

    /** Destructor */
    virtual ~MainWindow();

    enum textdisplay {
        OUT,
        REC,
        EXP
    };

    TalkingHead*        getFaceWidget();
    TextOutDisplay*     getTextWidget( textdisplay type );
    FestivalGenerator*  getGenerator();
    ImageDisplay* getImageStream();
//    ImageDisplay*       getImageDisplayer();


private slots:

private:
    int                     initialized_;
    float                   windowHeight_;
    float                   windowWidth_;

    TalkingHead*            talking_head_;


    TextOutDisplay*         expc_inp_display_;
    TextOutDisplay*         text_out_display_;
    TextOutDisplay*         text_rec_display_;

    FestivalGenerator*      festival_generator_;

    ImageDisplay*     image_stream_display_;

//    ImageDisplay*           image_display_;

    /** Load parameter from ROS config */
    //void loadParameterInTalkingHead(); 
    /** Initialize qt widgets for talking head, image display, text display, image stream display */
    int init();

};

#endif // TALKING_HEAD_INCLUDE_MAINWINDOW_H_
