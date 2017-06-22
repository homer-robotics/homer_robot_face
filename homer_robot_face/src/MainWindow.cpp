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
#include <homer_robot_face/MainWindow.h>

#include <string>
#include <vector>

//#include "tools/loadRosConfig.h"
#include <homer_robot_face/Config.h>

MainWindow::MainWindow( QWidget* parent ) : 
    QWidget( parent, Qt::FramelessWindowHint ), 
    talking_head_( 0 ),
    initialized_(false)
{
    ROS_INFO("Creating MainWindow..");

        //TODO FIXME
    windowHeight_ = 800;
    windowWidth_ = 600;


    int width = 600;
    int height = 800;
    int window_rotation = 0;

    QBoxLayout* center_layout;
    QBoxLayout* main_layout;
    QBoxLayout* h_layout;

    int max_width = width;
    int max_height = height;
    int min_width = width;
    int min_height = height;

    try
    {
        std::vector< std::vector<float> > material_colors;
        std::string filename = ros::package::getPath("homer_robot_face")+"/config/config.cfg";
        const char* cfgFilename = filename.c_str();

        Config cfg(cfgFilename);
        width = cfg.get("Window Width");
        height = cfg.get("Window Height");
        std::string mesh_filename = cfg.get("Mesh Filename");
        material_colors.push_back(cfg.get("Head Color"));
        material_colors.push_back(cfg.get("Iris Color"));
        material_colors.push_back(cfg.get("Outline Color"));
        window_rotation = cfg.get("Window Rotation");
        assert( window_rotation == 0 || window_rotation == 90 || window_rotation == 180 || window_rotation == 270 );
        talking_head_ = new TalkingHead( this, mesh_filename, material_colors, window_rotation );

        std::cout << "Mesh Filename : " << mesh_filename << std::endl;
    }
    catch( const std::exception& e )
    {
        std::cerr << e.what() << std::endl;
    }

    if( window_rotation == 0 || window_rotation == 180 )
    {
        center_layout = new QVBoxLayout();
        main_layout = new QVBoxLayout();
        h_layout = new QVBoxLayout();

        setMinimumSize( width, height );

        max_width = width/4;
        max_height = height/4;
        min_width = (width*3)/4;
        min_height = (height*3)/4;

        height = max_height;
        min_width = width;
    }
    else //if( window_rotation == 90 || window_rotation == 270 )
    {
        center_layout = new QHBoxLayout();
        main_layout = new QHBoxLayout();
        h_layout = new QHBoxLayout();
        int tmp = height;
        height = width;
        width = tmp;

        setMinimumSize( width, height );

        max_width = width/4;
        max_height = height/4;
        min_width = (width*3)/4;
        min_height = (height*3)/4;

        width = max_width;
        min_height = height;
    }

    setContentsMargins ( 0, 0, 0, 0 );

    QPalette custom_palette = palette();
    custom_palette.setColor ( QPalette::Window, Qt::black );
    custom_palette.setColor ( QPalette::WindowText, Qt::white );
    setPalette( custom_palette );
    setAutoFillBackground ( true );

    // Center Layout (fixed size to fit small screen)
    QWidget* center_widget = new QWidget();
    center_widget->setContentsMargins ( 0, 0, 0, 0 );

    center_layout->setSpacing( 0 );
    center_layout->setContentsMargins ( 0, 0, 0, 0 );

    festival_generator_ = new FestivalGenerator();

    //ImageStreamDisplay
    image_stream_display_ = new ImageDisplay( window_rotation, windowWidth_, windowHeight_ - 60, this );
    image_stream_display_->setMinimumSize(width,height-60);
    image_stream_display_->setMaximumSize( min_width, min_height-60 );


    // ExpectedInputDisplay
    expc_inp_display_= new TextOutDisplay( 0, 20, ::EXP, window_rotation, this);
    custom_palette.setColor ( QPalette::WindowText, QColor( 150, 150, 200 ) );
    expc_inp_display_->setPalette( custom_palette );
    expc_inp_display_->setMaximumSize( width, height );

    // TextOutputDisplay
    text_out_display_= new TextOutDisplay( 0, 26, ::OUT, window_rotation, this );
    text_out_display_->setMaximumSize( width, height );

    // TextRecognitionDisplay
    text_rec_display_= new TextOutDisplay( 0, 20, ::REC, window_rotation, this );
    custom_palette.setColor ( QPalette::WindowText, QColor( 200, 150, 150 ) );
    text_rec_display_->setPalette( custom_palette );
    text_rec_display_->setMaximumSize( width, height );

    // Main Layout
    main_layout->setSpacing( 0 );
    main_layout->setContentsMargins ( 0, 0, 0, 0 );

    if( window_rotation == 0 || window_rotation == 270 )
    {
        center_layout->addWidget( expc_inp_display_ );
        center_layout->addStretch();
        center_layout->addWidget( text_out_display_ );
        center_layout->addStretch();
        center_layout->addWidget( text_rec_display_ );

        center_layout->setStretchFactor( text_out_display_, 1 );

        //center_widget->setMinimumSize( width, height );
        center_widget->setMaximumSize( width, height);
        center_widget->setLayout( center_layout );

        main_layout->addWidget( talking_head_ );
        talking_head_->setMinimumSize( width, height );
        talking_head_->setMaximumSize( min_width, min_height );

        main_layout->setStretchFactor( talking_head_, 1 );

        main_layout->addWidget( image_stream_display_ );    //comment in
        main_layout->setStretchFactor( image_stream_display_, 1 );  //comment in

        h_layout->setSpacing( 0 );
        h_layout->setContentsMargins ( 0, 0, 0, 0 );

        h_layout->addStretch();
        h_layout->addWidget( center_widget );
        h_layout->addStretch();

        main_layout->addStretch();
        main_layout->addLayout( h_layout );
        main_layout->addStretch();
        std::cout << "end of main_layout stuff" << std::endl;
    }
    else //if( window_rotation == 90 || window_rotation == 180 )
    {
        center_layout->addWidget( text_rec_display_ );
        center_layout->addStretch();
        center_layout->addWidget( text_out_display_ );
        center_layout->addStretch();
        center_layout->addWidget( expc_inp_display_ );

        center_layout->setStretchFactor( text_out_display_, 1 );

        //center_widget->setMinimumSize( width, height );
        center_widget->setMaximumSize( width, height);
        center_widget->setLayout( center_layout );

        h_layout->setSpacing( 0 );
        h_layout->setContentsMargins ( 0, 0, 0, 0 );

        h_layout->addStretch();
        h_layout->addWidget( center_widget );
        h_layout->addStretch();

        main_layout->addStretch();
        main_layout->addLayout( h_layout );
        main_layout->addStretch();

        main_layout->addWidget( talking_head_ );
        talking_head_->setMinimumSize( width, height );
        talking_head_->setMaximumSize( min_width, min_height );

        main_layout->setStretchFactor( talking_head_, 1 );

        main_layout->addWidget( image_stream_display_ );
        main_layout->setStretchFactor( image_stream_display_, 1 );

    }
    //main_layout->setSizeConstraint(QLayout::SetFixedSize);  //TODO: toProve
    setLayout( main_layout );
    std::cout << "end of main window constructor" << std::endl;
    ostringstream next_command;
    next_command << "Info: " << qApp->desktop()->screenNumber( talking_head_ ) << endl;
    ROS_INFO( "%s", next_command.str().c_str() );
}

MainWindow::~MainWindow()
{
    if( talking_head_ ) delete talking_head_;
    if( festival_generator_ ) delete festival_generator_;
    if( image_stream_display_ ) delete image_stream_display_;
    if( text_out_display_ ) delete text_out_display_;
    if( text_rec_display_ ) delete text_rec_display_;
    if( expc_inp_display_ ) delete expc_inp_display_; 
//    if( image_display_ ) delete image_display_;
}

//void MainWindow::loadParameterInTalkingHead()
//{
//     std::string mesh_filename;

//     float head_r, head_g, head_b;
//     float iris_r, iris_g, iris_b;
//     float outline_r, outline_g, outline_b;

//     int window_rotation;

//     // read parameter from config (which is loaded to the parameter server)
//     loadConfigValue("/robot_face/sMeshFilename", mesh_filename);
//     //loadConfigValue("/robot_face/vHeadColor", head_colors);
//     loadConfigValue("/robot_face/fHeadColorR", head_r);
//     loadConfigValue("/robot_face/fHeadColorG", head_g);
//     loadConfigValue("/robot_face/fHeadColorB", head_b);
//     //loadConfigValue("/robot_face/vIrisColor", iris_colors);
//     loadConfigValue("/robot_face/fIrisColorR", iris_r);
//     loadConfigValue("/robot_face/fIrisColorG", iris_g);
//     loadConfigValue("/robot_face/fIrisColorB", iris_b);
//     //loadConfigValue("/robot_face/vOutlineColor", outline_colors);
//     loadConfigValue("/robot_face/fOutlineColorR", outline_r);
//     loadConfigValue("/robot_face/fOutlineColorG", outline_g);
//     loadConfigValue("/robot_face/fOutlineColorB", outline_b);
//     loadConfigValue("/robot_face/iHeight", windowHeight_); //loaded in member
//     loadConfigValue("/robot_face/iWidth", windowWidth_);   //loaded in member
//     loadConfigValue("/robot_face/iRotation", window_rotation);


//     assert( window_rotation == 0 || window_rotation == 90 || window_rotation == 180 || window_rotation == 270 );

//     std::vector<float> head_colors;
//     head_colors.push_back(head_r);
//     head_colors.push_back(head_g);
//     head_colors.push_back(head_b);
//     std::vector<float> iris_colors;
//     iris_colors.push_back(iris_r);
//     iris_colors.push_back(iris_g);
//     iris_colors.push_back(iris_b);
//     std::vector<float> outline_colors;
//     outline_colors.push_back(outline_r);
//     outline_colors.push_back(outline_g);
//     outline_colors.push_back(outline_b);

//     std::vector< std::vector<float> > material_colors;
//     material_colors.push_back(head_colors);
//     material_colors.push_back(iris_colors);
//     material_colors.push_back(outline_colors);

//     talking_head_ = new TalkingHead( this, mesh_filename, material_colors, window_rotation );
//}

int MainWindow::init()
{
     if(initialized_)
         ROS_WARN("MainWindow has been already initialized. Reinitialize..");

     ROS_INFO("Initialize MainWindow..");

     //this->loadParameterInTalkingHead();

     int width = windowWidth_;
     int height = windowHeight_;
     int window_rotation = talking_head_->getWindowRotation();

     int max_width = width;
     int max_height = height;
     int min_width = width;
     int min_height = height;

     QBoxLayout* center_layout;
     QBoxLayout* main_layout;
     QBoxLayout* h_layout;

     ostringstream meshFilenameStr;
     meshFilenameStr << "Info: " << qApp->desktop()->screenNumber( talking_head_ ) << endl;
     ROS_INFO( "%s", meshFilenameStr.str().c_str() );

     if( window_rotation == 0 || window_rotation == 180 )
     {
         center_layout = new QVBoxLayout();
         main_layout = new QVBoxLayout();
         h_layout = new QVBoxLayout();

         setMinimumSize( width, height );

         max_width = width/4;
         max_height = height/4;
         min_width = (width*3)/4;
         min_height = (height*3)/4;

         height = max_height;
         min_width = width;
     }
     else //if( window_rotation == 90 || window_rotation == 270 )
     {
         center_layout = new QHBoxLayout();
         main_layout = new QHBoxLayout();
         h_layout = new QHBoxLayout();
         int tmp = height;
         height = width;
         width = tmp;

         setMinimumSize( width, height );

         max_width = width/4;
         max_height = height/4;
         min_width = (width*3)/4;
         min_height = (height*3)/4;

         width = max_width;
         min_height = height;
     }

     setContentsMargins ( 0, 0, 0, 0 );

     QPalette custom_palette = palette();
     custom_palette.setColor ( QPalette::Window, Qt::black );
     custom_palette.setColor ( QPalette::WindowText, Qt::white );
     setPalette( custom_palette );
     setAutoFillBackground ( true );

     // Center Layout (fixed size to fit small screen)
     QWidget* center_widget = new QWidget();
     center_widget->setContentsMargins ( 0, 0, 0, 0 );

     center_layout->setSpacing( 0 );
     center_layout->setContentsMargins ( 0, 0, 0, 0 );

 //    //If we have more than one screen, open full-screen on 2nd screen
 //    ostringstream command;
 //    command << "Number of screens: " << qApp->desktop()->numScreens() << endl
 //            << "Screennr.: " << qApp->desktop()->screen(0) << endl
 //            << "Screennr.: " << qApp->desktop()->screen(1) << endl
 //            << "Primary screen: " << qApp->desktop()->primaryScreen() << endl
 //            << "Is virtual desktop: " << qApp->desktop()->isVirtualDesktop() << endl
 //            <<  "screen 0: " << qApp->desktop()->availableGeometry(0).left() << ","
 //            << qApp->desktop()->availableGeometry(0).top() << ","
 //            << qApp->desktop()->availableGeometry(0).right() << ","
 //            << qApp->desktop()->availableGeometry(0).bottom() << endl
 //            <<  "screen 1: " << qApp->desktop()->availableGeometry(1).left() << ","
 //            << qApp->desktop()->availableGeometry(1).top() << ","
 //            << qApp->desktop()->availableGeometry(1).right() << ","
 //            << qApp->desktop()->availableGeometry(1).bottom() << ",";

 //    ROS_INFO( command.str().c_str() );

     festival_generator_ = new FestivalGenerator();

     //ImageStreamDisplay
     image_stream_display_ = new ImageDisplay( window_rotation, windowWidth_, windowHeight_ - 60, this );
     image_stream_display_->setMinimumSize(width,height);
     image_stream_display_->setMaximumSize( min_width, min_height );

     // ExpectedInputDisplay
     expc_inp_display_= new TextOutDisplay( 0, 20, ::EXP, window_rotation, this);
     custom_palette.setColor ( QPalette::WindowText, QColor( 150, 150, 200 ) );
     expc_inp_display_->setPalette( custom_palette );
     expc_inp_display_->setMaximumSize( width, height );

     // TextOutputDisplay
     text_out_display_= new TextOutDisplay( 0, 26, false, window_rotation, this );
     text_out_display_->setMaximumSize( width, height );

     // TextRecognitionDisplay
     text_rec_display_= new TextOutDisplay( 0, 20, true, window_rotation, this );
     custom_palette.setColor ( QPalette::WindowText, QColor( 200, 150, 150 ) );
     text_rec_display_->setPalette( custom_palette );
     text_rec_display_->setMaximumSize( width, height );

     // Main Layout
     main_layout->setSpacing( 0 );
     main_layout->setContentsMargins ( 0, 0, 0, 0 );

     if( window_rotation == 0 || window_rotation == 270 )
     {
         center_layout->addWidget( expc_inp_display_ );
         center_layout->addStretch();
         center_layout->addWidget( text_out_display_ );
         center_layout->addStretch();
         center_layout->addWidget( text_rec_display_ );

         center_layout->setStretchFactor( text_out_display_, 1 );

         center_widget->setMinimumSize( width, height );
         center_widget->setMaximumSize( width, height);
         center_widget->setLayout( center_layout );

         main_layout->addWidget( talking_head_ );
         talking_head_->setMinimumSize( width, height );
         talking_head_->setMaximumSize( min_width, min_height );

         main_layout->setStretchFactor( talking_head_, 1 );

         main_layout->addWidget( image_stream_display_ );
         main_layout->setStretchFactor( image_stream_display_, 1 );

         h_layout->setSpacing( 0 );
         h_layout->setContentsMargins ( 0, 0, 0, 0 );

         h_layout->addStretch();
         h_layout->addWidget( center_widget );
         h_layout->addStretch();

         main_layout->addStretch();
         main_layout->addLayout( h_layout );
         main_layout->addStretch();
     }
     else //if( window_rotation == 90 || window_rotation == 180 )
     {
         center_layout->addWidget( text_rec_display_ );
         center_layout->addStretch();
         center_layout->addWidget( text_out_display_ );
         center_layout->addStretch();
         center_layout->addWidget( expc_inp_display_ );

         center_layout->setStretchFactor( text_out_display_, 1 );

         center_widget->setMinimumSize( width, height );
         center_widget->setMaximumSize( width, height);
         center_widget->setLayout( center_layout );

         h_layout->setSpacing( 0 );
         h_layout->setContentsMargins ( 0, 0, 0, 0 );

         h_layout->addStretch();
         h_layout->addWidget( center_widget );
         h_layout->addStretch();

         main_layout->addStretch();
         main_layout->addLayout( h_layout );
         main_layout->addStretch();

         main_layout->addWidget( talking_head_ );
         talking_head_->setMinimumSize( width, height );
         talking_head_->setMaximumSize( min_width, min_height );

         main_layout->setStretchFactor( talking_head_, 1 );

         main_layout->addWidget( image_stream_display_ );
         main_layout->setStretchFactor( image_stream_display_, 1 );

     }

     setLayout( main_layout );

     ostringstream next_command;
     next_command << "Info: " << qApp->desktop()->screenNumber( talking_head_ ) << endl;
     ROS_INFO( "%s", next_command.str().c_str() );


     initialized_ = true;
     ROS_INFO("MainWindow initialization successfull!");
    return 1;
}

TalkingHead* MainWindow::getFaceWidget()
{
    return talking_head_;
}

TextOutDisplay* MainWindow::getTextWidget( textdisplay type )
{
    if( type == OUT )
    {
        return text_out_display_;
    }
    if( type == REC )
    {
        return text_rec_display_;
    }
    if( type == EXP )
    {
        return expc_inp_display_;
    }
    return text_out_display_;
}

FestivalGenerator* MainWindow::getGenerator()
{
    return festival_generator_;
}

ImageDisplay* MainWindow::getImageStream()
{
    return image_stream_display_;
}
