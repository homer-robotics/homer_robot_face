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

#include <homer_robot_face/TalkingHead.h>
#include <homer_robot_face/TextProcessor.h>

//#include "MainWindow.h" //needed for checking on processed images for visualize stream

#if defined(Q_WS_WIN)
#include <windows.h>  // needed for WindowFromDC()
#else
#include <QX11Info>
#include <X11/Xlib.h>
#endif

#include <map>
#include <string>
#include <utility>
#include <vector>

TalkingHead::TalkingHead( QWidget* parent, std::string mesh_string, std::vector< std::vector<float> > material_vector, int window_rotation ) :
    QWidget( parent ),
    phones_( 0 ),
    words_( 0 ),
    root_( 0 ),
    camera_( 0 ),
    viewport_( 0 ),
    scene_manager_( 0 ),
    shut_down_( false ),
    window_( 0 ),
    pose_list_( 0 ),
    num_sub_meshes_( 0 ),
    anim_set_( 0 ),
    mouth_animation_state_( 0 ),
    mouth_animation_state_name_( "mouthAnimationState" ),
    eyebrows_animation_state_( 0 ),
    eyebrows_animation_state_name_( "eyebrowsAnimationState" ),
    vertex_animation_tracks_( 0 ),
    manual_key_frame_( 0 ),
    visemes_arrived_( false ),
    words_arrived_( false ),
    text_for_emotion_( " " ),
    is_visible_( false ),
    show_time_( 1 ),
    angry_(false),
    smile_(false),
    sad_(false),
    rest_(false),
    surprised_(false),
    disgusted_(false),
    afraid_(false)
{
    init_ = true;
    setMinimumSize( 100, 100 );
    redraw_timer_ = new QTimer ( this );  // create internal timer
    connect ( redraw_timer_, SIGNAL ( timeout() ), SLOT ( updateOverlay() ) );
    connect ( this, SIGNAL( timerChanged(int) ), SLOT( setTimer(int) ) );
    //redraw_timer_->start( 1 );

    connect( this, SIGNAL(faceCleared()), SLOT( clearFace()) );

    //setVisible( false );
    redraw_timer_->start( 3001 );

    initPhoneMap();
    srand( ( unsigned )time( 0 ) );

    smileys_.push_back( ">:" );
    smileys_.push_back(":O");
    smileys_.push_back(":o");
    smileys_.push_back(":!");
    smileys_.push_back( ":)" );
    smileys_.push_back( ":(" );
    smileys_.push_back( "." );
    smileys_.push_back( ":&" );


    mesh_string_ = mesh_string;
    material_vector_ = material_vector;
    window_rotation_ = window_rotation;
}
//---------------------------------------------------------------------------

TalkingHead::~TalkingHead()
{
    remove( "phonemes.txt" );
    remove( "words.txt" );
    if( redraw_timer_ ) delete redraw_timer_;
    Ogre::WindowEventUtilities::removeWindowEventListener( window_, this );
    windowClosed( window_ );
    if( root_ ) delete root_;
}
//---------------------------------------------------------------------------

void TalkingHead::updateOverlay()
{
    if( init_ )
    {
        ROS_INFO("hallo talkinghead");
        emit timerChanged( 1 );
        init_ = false;
        is_visible_ = true;
    }
    setVisible( is_visible_ );
    Ogre::Root::getSingleton()._fireFrameStarted();
    window_->update();
    Ogre::Root::getSingleton()._fireFrameRenderingQueued();
    Ogre::Root::getSingleton()._fireFrameEnded();

    update();
}

void TalkingHead::showEvent( QShowEvent *event )
{
    if ( !root_ )
    {
        initOgreSystem();
    }

    QWidget::showEvent( event );
}
//---------------------------------------------------------------------------

void TalkingHead::paintEvent( QPaintEvent *event )
{
    Ogre::Root::getSingleton()._fireFrameStarted();
    window_->update();
    Ogre::Root::getSingleton()._fireFrameRenderingQueued();
    Ogre::Root::getSingleton()._fireFrameEnded();

    update();
}
//---------------------------------------------------------------------------

void TalkingHead::moveEvent( QMoveEvent * event )
{
    QWidget::moveEvent( event );

    if( event->isAccepted() && window_ )
    {
        window_->windowMovedOrResized();
        update();
    }
}
//---------------------------------------------------------------------------

void TalkingHead::resizeEvent( QResizeEvent *event )
{
    QWidget::resizeEvent( event );

    if( event->isAccepted() )
    {
        const QSize &newSize = event->size();
        if( window_ )
        {
            window_->resize( newSize.width(), newSize.height() );
            window_->windowMovedOrResized();
        }
        if( camera_ )
        {
            Ogre::Real aspectRatio = Ogre::Real( newSize.width() ) / Ogre::Real( newSize.height() );
            camera_->setAspectRatio( aspectRatio );
        }
    }
}
//---------------------------------------------------------------------------

void TalkingHead::initOgreSystem( void )
{
    try
    {
    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog( "Ogre.log", true, false, true );

    std::string plugin_prefix;
#ifdef OGRE_PLUGIN_PATH
    plugin_prefix = OGRE_PLUGIN_PATH + std::string( "/" );
#endif

    root_ = new Ogre::Root();
    root_->loadPlugin( plugin_prefix + "RenderSystem_GL" );

    // Taken from gazebo
    Ogre::RenderSystem* render_system = NULL;
#if OGRE_VERSION_MAJOR >=1 && OGRE_VERSION_MINOR >= 7
    Ogre::RenderSystemList rs_list = root_->getAvailableRenderers();
    Ogre::RenderSystemList::iterator render_it = rs_list.begin();
    Ogre::RenderSystemList::iterator render_end_it = rs_list.end();
#else
    Ogre::RenderSystemList* rs_list = root_->getAvailableRenderers();
    Ogre::RenderSystemList::iterator render_it = rs_list->begin();
    Ogre::RenderSystemList::iterator render_end_it = rs_list->end();
#endif
    for ( ; render_it != render_end_it; ++render_it )
    {
      render_system = *render_it;

      if ( render_system->getName() == "OpenGL Rendering Subsystem" )
      {
        break;
      }
    }

    if ( render_system == NULL )
    {
      throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
    }

    render_system->setConfigOption( "Full Screen", "No" );
    render_system->setConfigOption( "FSAA", "16" );
    render_system->setConfigOption( "RTT Preferred Mode", "FBO" );
    render_system->setConfigOption( "VSync", "Yes" );
    render_system->setConfigOption( "sRGB Gamma Conversion", "No" );

    root_->setRenderSystem( render_system );

    root_->initialise( false );

    std::string ogre_tools_path = ros::package::getPath( ROS_PACKAGE_NAME );

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/mesh", "FileSystem", mesh_string_ );
    }
    catch( const std::exception& e )
    {
      printf( "Failed to initialize Ogre: %s\n", e.what() );
      throw;
    }

    root_->restoreConfig();

    Ogre::NameValuePairList viewConfig;
    Ogre::String winHandle;

    viewConfig[ "monitorIndex" ] = "2";
    winHandle = Ogre::StringConverter::toString ( (unsigned long)QX11Info::display() ) +
           ":" + Ogre::StringConverter::toString ( (unsigned int)QX11Info::appScreen() ) +
           ":" + Ogre::StringConverter::toString ( (unsigned long)nativeParentWidget()->effectiveWinId() );

    viewConfig[ "externalWindowHandle" ] = winHandle;
    viewConfig[ "FSAA" ] = "16";
    viewConfig[ "vsync" ] = "true";
    viewConfig[ "border" ] = "none";
    window_ = root_->createRenderWindow( "RobotFace", width(), height(), false, &viewConfig );
    window_->setActive( true );
    WId ogreWinId = 0x0;
    window_->getCustomAttribute( "WINDOW", &ogreWinId );

    assert( ogreWinId );

    QWidget::create( ogreWinId );
    setAttribute( Qt::WA_PaintOnScreen, true );
    setAttribute( Qt::WA_OpaquePaintEvent );

    // Create SceneManager
    scene_manager_ = root_->createSceneManager( Ogre::ST_GENERIC );

    // Create Camera
    camera_ = scene_manager_->createCamera( "Camera" );

    camera_->setPosition( Ogre::Vector3( 0, 0, 12 ) );
    camera_->lookAt( Ogre::Vector3( 0, 0, 1 ) );
    camera_->setNearClipDistance( 1 );

    // Create one viewport, entire window
    viewport_ = window_->addViewport( camera_ );
    viewport_->setBackgroundColour( Ogre::ColourValue ( 0, 0, 0 ) );

    // Alter the camera aspect ratio to match the viewport
    camera_->setAspectRatio( Ogre::Real( width() / height() ) );

    // Load Resources
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup( mesh_string_ );

    // Pre-load the mesh so that we can tweak it with a manual animation
    createAnimations( mesh_string_ + ".mesh" );

    // Create Scene
    Ogre::Entity* head = scene_manager_->createEntity( "Head", mesh_string_ + ".mesh" );
    Ogre::SceneNode* headNode = scene_manager_->getRootSceneNode()->createChildSceneNode( "HeadNode", Ogre::Vector3( 0.0f, 0.0f, 0.0f ) );
    headNode->attachObject( head );

    if(window_rotation_ == 90) window_rotation_ = -90;
    if(window_rotation_ == 270) window_rotation_ = -270;

    headNode->rotate(Ogre::Quaternion(Ogre::Degree( window_rotation_ ), Ogre::Vector3::UNIT_Z), headNode->TS_WORLD);

    // Create Animations
    anim_set_ = head->getAllAnimationStates();

    mouth_animation_state_ = head->getAnimationState( mouth_animation_state_name_ );

    eyebrows_animation_state_ = head->getAnimationState( eyebrows_animation_state_name_ );

    initAnimationStates();

    // Set ambient light
    scene_manager_->setAmbientLight( Ogre::ColourValue( 0.5, 0.5, 0.5 ) );

    // Create spotlight
    Ogre::Light* spotLight = scene_manager_->createLight("spotLight");
    spotLight->setType(Ogre::Light::LT_SPOTLIGHT);

    spotLight->setDirection(0, 0, -1);
    spotLight->setPosition(Ogre::Vector3(0, 0, 10));

    spotLight->setSpotlightRange(Ogre::Degree(10), Ogre::Degree(20));

    // Create a light
    Ogre::Light* light = scene_manager_->createLight( "Light" );
    light->setPosition( 20, 80, 50 );

    windowResized( window_ );

    Ogre::WindowEventUtilities::addWindowEventListener( window_, this );

    root_->addFrameListener( this );

    while( ros::ok() )
    {
        // Render next frame
        if( !root_->renderOneFrame() )
        {
           root_->getRenderSystem()->shutdown();
           ros::shutdown();
        }

        if( window_->isClosed() )
        {
           root_->getRenderSystem()->shutdown();
           ros::shutdown();
        }

        window_->update();
        //Ogre::WindowEventUtilities::messagePump();
    }

    return;
}
//---------------------------------------------------------------------------

bool TalkingHead::frameRenderingQueued( const Ogre::FrameEvent& evt )
{
    bool ret = Ogre::FrameListener::frameRenderingQueued( evt );

    if( window_->isClosed() )
        return false;

    if( shut_down_ )
        return false;

    Ogre::AnimationStateIterator animations_it = anim_set_->getAnimationStateIterator();

    while (animations_it.hasMoreElements())
    {
        Ogre::AnimationState *animationState = animations_it.getNext();
        if( (visemes_arrived_) && animationState->getAnimationName() == mouth_animation_state_name_ )
        {
            animationState->addTime( evt.timeSinceLastFrame );
            // update influence of a pose reference
            updatePoses( "mouth", 0 );
        }

        if( (visemes_arrived_) && animationState->getAnimationName() == eyebrows_animation_state_name_ )
        {
            animationState->addTime( evt.timeSinceLastFrame );
            // update influence of a pose reference
            updatePoses( "eyebrows", 0 );
        }

        if( animationState->getAnimationName() == "breathe" )
        {
            // enable breathing
            animationState->addTime( evt.timeSinceLastFrame );
        }

        if( animationState->getAnimationName() == "blink" )
        {
            // enable blinking
            animationState->addTime( evt.timeSinceLastFrame );
            // reset current blinking
            if ( root_->getTimer()->getMilliseconds() / 1000 % (rand()/RAND_MAX * (6 - 4 + 1) + 4) == 0 )
            {
                animationState->setTimePosition( 0 );
            }
        }

        if( animationState->getAnimationName() == "rotate" )
        {
            // enable wiggle
            animationState->addTime( evt.timeSinceLastFrame );
            // reset current wiggle
            if( root_->getTimer()->getMilliseconds() / 1000 % (rand()/RAND_MAX * (22 - 12 + 1) + 12) == 0 )
            {
                animationState->setTimePosition( 0 );
            }
        }
    }

    setVisible( is_visible_ );

    return ret;
}
//---------------------------------------------------------------------------

void TalkingHead::createAnimations( std::string mesh_file )
{
    if( !material_vector_.empty() )
    {
        changeMaterialColor();
    }

    mesh_ = Ogre::MeshManager::getSingleton().load( mesh_file, mesh_string_ );

    try
    {
        bool needVertexAnimation = true;
        bool needSkeletalAnimation = false;

        bool mouth = false;
        bool eyebrows = false;

        pose_list_ = mesh_->getPoseList();

        num_sub_meshes_ = mesh_->getNumSubMeshes();

        Ogre::Animation* mouthAnimation = mesh_->createAnimation( mouth_animation_state_name_, 0 );
        Ogre::Animation* eyebrowsAnimation = mesh_->createAnimation( eyebrows_animation_state_name_, 0 );

        for( int curSubMesh = 1; curSubMesh <= num_sub_meshes_; curSubMesh++ )
        {
            Ogre::VertexData* vertexData = mesh_->getVertexDataByTrackHandle( curSubMesh );  // main submesh
            Ogre::VertexDeclaration* newDeclaration = vertexData->vertexDeclaration->getAutoOrganisedDeclaration( needSkeletalAnimation, needVertexAnimation, false );
            vertexData->reorganiseBuffers( newDeclaration );

            for( unsigned int curPoseIndex = 0; curPoseIndex < mesh_->getPoseCount(); curPoseIndex++ )
            {
                if( atof( Ogre::StringUtil::split( mesh_->getPose( curPoseIndex )->getName(), "-", 3 ).at( 2 ).c_str() ) == curSubMesh-1
                && Ogre::StringUtil::startsWith( mesh_->getPose( curPoseIndex )->getName(), "mouth", true )
                && mouth == false )
                {
                        Ogre::VertexAnimationTrack* vat = mouthAnimation->createVertexTrack( curSubMesh,  Ogre::VAT_POSE );
                        mouth = true;
                        vertex_animation_tracks_.push_back( vat );
                }
               if( atof( Ogre::StringUtil::split( mesh_->getPose( curPoseIndex )->getName(), "-", 3 ).at( 2 ).c_str() ) == curSubMesh-1
                && Ogre::StringUtil::startsWith( mesh_->getPose( curPoseIndex )->getName(), "eyebrows", true )
                && eyebrows == false )
                {
                        Ogre::VertexAnimationTrack* vat = eyebrowsAnimation->createVertexTrack( curSubMesh,  Ogre::VAT_POSE );
                        eyebrows = true;
                        vertex_animation_tracks_.push_back( vat );
                }
            }
        }
    }
    catch( std::runtime_error err )
    {
        std::string error = "ERROR: ";
        error.append( __FILE__ );
        error.append( err.what() );
        Ogre::LogManager::getSingleton().logMessage( error );
    }
}
//---------------------------------------------------------------------------

void TalkingHead::changeMaterialColor()
{
    std::vector<float> head_color = material_vector_.at(0);
    std::vector<float> iris_color = material_vector_.at(1);
    std::vector<float> outline_color = material_vector_.at(2);

    material_ = Ogre::MaterialManager::getSingleton().load("Head", mesh_string_).staticCast<Ogre::Material>();
    Ogre::Technique* tech = material_->createTechnique();
    Ogre::Pass* pass = tech->createPass();
    pass = material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(Ogre::ColourValue(head_color.at(0), head_color.at(1), head_color.at(2), 1.0));

    material_ = Ogre::MaterialManager::getSingleton().load("Iris", mesh_string_).staticCast<Ogre::Material>();
    tech = material_->createTechnique();
    pass = tech->createPass();
    pass = material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(Ogre::ColourValue(iris_color.at(0), iris_color.at(1), iris_color.at(2), 1.0));

    material_ = Ogre::MaterialManager::getSingleton().load("Mouth", mesh_string_).staticCast<Ogre::Material>();
    tech = material_->createTechnique();
    pass = tech->createPass();
    pass = material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(Ogre::ColourValue(outline_color.at(0), outline_color.at(1), outline_color.at(2), 1.0));

    material_ = Ogre::MaterialManager::getSingleton().load("EyeBrows", mesh_string_).staticCast<Ogre::Material>();
    tech = material_->createTechnique();
    pass = tech->createPass();
    pass = material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(Ogre::ColourValue(outline_color.at(0), outline_color.at(1), outline_color.at(2), 1.0));

    material_ = Ogre::MaterialManager::getSingleton().load("Eyes", mesh_string_).staticCast<Ogre::Material>();
    tech = material_->createTechnique();
    pass = tech->createPass();
    pass = material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(Ogre::ColourValue(outline_color.at(0), outline_color.at(1), outline_color.at(2), 1.0));
}
//---------------------------------------------------------------------------

void TalkingHead::initAnimationStates()
{
    Ogre::AnimationStateIterator animations_it = anim_set_->getAnimationStateIterator();

    // For each animation
    while (animations_it.hasMoreElements())
    {
        Ogre::AnimationState *a = animations_it.getNext();
        a->setEnabled( true );
        a->setLoop( false );
        if( a->getAnimationName() == "breathe" )
        {
           a->setLoop( true );
        }
        a->setTimePosition( 0 );
    }
}
//---------------------------------------------------------------------------

void TalkingHead::initPhoneMap()
{
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "aa", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ae", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ah", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ao", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "aw", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ax", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "axr", "open" ));
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ay", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "b", "close" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ch", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "d", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "dh", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "dx", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "eh", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "el", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "em", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "en", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "er", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ey", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "f", "close" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "g", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "hh", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "hv", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ih", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "iy", "wide" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "jh", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "k", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "l", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "m", "close" ));
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "n", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "nx", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ng", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "ow", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "oy", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "p", "close" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "r", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "s", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "sh", "open" ));
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "t", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "th", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "uh", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "uw", "narrow" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "v", "close" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "w", "close" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "y", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "z", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "zh", "open" ) );
    phonemes_.insert( std::pair<Ogre::String, Ogre::String>( "pau", "pau" ) );
}
//---------------------------------------------------------------------------

std::map<float, Ogre::String> TalkingHead::createVisemeMap()
{
    std::map<Ogre::String, Ogre::String>::iterator phoneme_it = phonemes_.begin();

    std::map<float, Ogre::String> visemes;

    Ogre::String l;
    Ogre::String phone = "";
    float timeStamp;

    for(unsigned int i = 1; i < phones_.size(); i++)
    {
        l = phones_.at(i);

        timeStamp = atof( Ogre::StringUtil::split( l, " ", 3 ).at( 0 ).c_str() );
        phone = Ogre::StringUtil::split( l, " ", 3 ).at( 2 );

        phoneme_it = phonemes_.find( phone );

        visemes.insert( std::pair<float, Ogre::String>( timeStamp, phoneme_it->second ) );
    }

    return visemes;
}
//---------------------------------------------------------------------------

std::map<float, Ogre::String> TalkingHead::createWordMap()
{
    std::map<float, Ogre::String> words;

    Ogre::String l;
    Ogre::String word = "";
    float timeStamp;

    for(unsigned int i = 1; i < words_.size(); i++)
    {
        l = words_.at(i);

        timeStamp = atof( Ogre::StringUtil::split( l, " ", 3 ).at( 0 ).c_str() );
        word = Ogre::StringUtil::split( l, " ", 3 ).at( 2 );

        words.insert( std::pair<float, Ogre::String>( timeStamp, word ) );
    }

    return words;
}
//---------------------------------------------------------------------------

void TalkingHead::playTalkAnimation()
{
    std::map<float, Ogre::String>::iterator words_it = word_map_.begin();

    if( visemes_arrived_ )
    {
        for( unsigned int curVAT = 0; curVAT < vertex_animation_tracks_.size(); curVAT++ )
        {
            vertex_animation_tracks_[ curVAT ]->removeAllKeyFrames();
        }
        visemes_arrived_ = false;
    }

    for( std::map<float, Ogre::String>::iterator visemes_it = viseme_map_.begin(); visemes_it != viseme_map_.end(); ++visemes_it)
    {
        mouth_animation_state_->setTimePosition( 0 );
        mouth_animation_state_->setLength( visemes_it->first );
        eyebrows_animation_state_->setTimePosition( 0 );
        eyebrows_animation_state_->setLength( visemes_it->first );

        if( words_it->first == visemes_it->first )
        {
            std::vector<std::string> spaces;
            spaces.push_back("");
            spaces.push_back(" ");

            size_t i_smiley = std::string::npos;

            for( unsigned int i = 0; i < spaces.size(); i++)
            {
                for( unsigned int j = 0; j < smileys_.size(); j++ )
                {
                    i_smiley = text_for_emotion_.find( words_it->second + spaces.at(i) + smileys_.at( j ), 0 );
                    if( i_smiley != std::string::npos && (smileys_.at( j ) == ":)") )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        smile_ = true;
                        angry_ = false;
                        sad_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":(" ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        sad_ = true;
                        angry_ = false;
                        smile_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ">:" ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        angry_ = true;
                        sad_ = false;
                        smile_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == "." ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        rest_ = true;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":O" || smileys_.at( j ) == ":o") )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = true;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":!") )
                    {
                        afraid_ = false;
                        disgusted_ = true;
                        surprised_ = false;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":&") )
                    {
                        afraid_ = true;
                        disgusted_ = false;
                        surprised_ = false;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    i_smiley = text_for_emotion_.find( smileys_.at( j ) + spaces.at(i) + words_it->second, 0 );
                    if( i_smiley != std::string::npos && (smileys_.at( j ) == ":)") )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        smile_ = true;
                        angry_ = false;
                        sad_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":(" ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        sad_ = true;
                        angry_ = false;
                        smile_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ">:" ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        angry_ = true;
                        sad_ = false;
                        smile_ = false;
                        rest_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == "." ) )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = false;
                        rest_ = true;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":O" || smileys_.at( j ) == ":o") )
                    {
                        afraid_ = false;
                        disgusted_ = false;
                        surprised_ = true;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":!") )
                    {
                        afraid_ = false;
                        disgusted_ = true;
                        surprised_ = false;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                    else if( i_smiley != std::string::npos && (smileys_.at( j ) == ":&") )
                    {
                        afraid_ = true;
                        disgusted_ = false;
                        surprised_ = false;
                        rest_ = false;
                        sad_ = false;
                        smile_ = false;
                        angry_ = false;
                    }
                }
            }

            text_for_emotion_.erase(0, (words_it->second).length());

            if( words_it != word_map_.end() )
                words_it++;
        }

        for( unsigned int curVAT = 0; curVAT < vertex_animation_tracks_.size(); curVAT++ )
        {
            if( vertex_animation_tracks_[ curVAT ]->getParent()->getName() == mouth_animation_state_name_ )
            {
                vertex_animation_tracks_[ curVAT ]->getParent()->setLength( visemes_it->first );

                if( (words_it->first == visemes_it->first) && (smile_ || sad_ || angry_ || rest_ || surprised_ || disgusted_))
                {
                    manual_key_frame_ = vertex_animation_tracks_[ curVAT ]->createVertexPoseKeyFrame( visemes_it->first - 0.5 );
                }
                else
                {
                    manual_key_frame_ = vertex_animation_tracks_[ curVAT ]->createVertexPoseKeyFrame( visemes_it->first );
                }

                for( unsigned int curPoseIndex = 0; curPoseIndex < mesh_->getPoseCount(); curPoseIndex++ )
                {
                    float influence = static_cast<float>(rand()) / static_cast<float>((RAND_MAX * (1.0 - 0.75 + 1) + 0.75));  // rand() / (RAND_MAX * (nHigh - nLow + 1)) + nLow

                    if( visemes_it->second == "wide" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-wide", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( visemes_it->second == "narrow" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-narrow", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( visemes_it->second == "open" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-open", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( visemes_it->second == "close" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-close", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( smile_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-smile", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.85 );
                    }
                    if( sad_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-sad", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.85 );
                    }
                    if( angry_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-sad", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.85 );
                    }
                    if( rest_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-rest", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 1.0 );
                    }
                    if( surprised_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-narrow", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.5 );
                    }
                    if( surprised_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-open", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.5 );
                    }
                    if( disgusted_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-disgusted", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 1.0 );
                    }
                    if( afraid_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-open", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 0.9 );
                    }
                    if( afraid_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-afraid", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, 1.0 );
                    }
                    if( visemes_it->second == "pau" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "mouth-rest", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                }
            }
            if( vertex_animation_tracks_[ curVAT ]->getParent()->getName() == eyebrows_animation_state_name_ )
            {
                vertex_animation_tracks_[ curVAT ]->getParent()->setLength( visemes_it->first );

                if( (words_it->first == visemes_it->first) && (smile_ || sad_ || angry_ || rest_ || surprised_ || disgusted_ ))
                {
                    manual_key_frame_ = vertex_animation_tracks_[ curVAT ]->createVertexPoseKeyFrame( visemes_it->first - 0.5 );
                }
                else
                {
                    manual_key_frame_ = vertex_animation_tracks_[ curVAT ]->createVertexPoseKeyFrame( visemes_it->first );
                }

                for( unsigned int curPoseIndex = 0; curPoseIndex < mesh_->getPoseCount(); curPoseIndex++ )
                {
                    float influence = 0.9;

                    if( rest_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-rest", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( visemes_it->second == "pau" && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-rest", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( smile_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-up", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( sad_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-sad", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( angry_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-down", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( surprised_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-up", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( disgusted_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-frown", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( afraid_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-up", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( afraid_ && Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-frown", true ) )
                    {
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                    }
                    if( Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), "eyebrows-rest", true ) )
                        manual_key_frame_->addPoseReference( curPoseIndex, influence );
                }

            }
        }
    }

    visemes_arrived_ = true;
}
//---------------------------------------------------------------------------*/

void TalkingHead::updatePoses( Ogre::String pose_name, float weight )
{
    int curPoseIndex = 0;

    for( int curSubMesh = 1; curSubMesh <= num_sub_meshes_; curSubMesh++ )
    {
        while( pose_list_[ curPoseIndex ]->getTarget() == curSubMesh-1 )
        {
            if( Ogre::StringUtil::startsWith( pose_list_[ curPoseIndex ]->getName(), pose_name, true ) )
            {
              if(manual_key_frame_)
              {
                    manual_key_frame_->updatePoseReference( curPoseIndex, weight );
              }
            }
            curPoseIndex++;
        }
    }
}
//---------------------------------------------------------------------------

void TalkingHead::callbackVisemes()
{
    words_.clear();
    phones_.clear();

    std::ifstream wordsfile( "words.txt" );

    if( wordsfile )
    {
        Ogre::String line;

        while( getline( wordsfile, line ) )
        {
            words_.push_back( line );
        }

        wordsfile.close();
        remove( "words.txt" );

        word_map_ = createWordMap();
    }

    std::ifstream phonemesfile( "phonemes.txt" );

    if( phonemesfile )
    {
        Ogre::String line;

        while( getline( phonemesfile, line ) )
        {
            phones_.push_back( line );
        }

        phonemesfile.close();
        remove( "phonemes.txt" );

        viseme_map_ = createVisemeMap();

        playTalkAnimation();
    }

    if(word_map_.empty() && viseme_map_.empty())
    {
        word_map_.insert( std::pair<float, Ogre::String>( 0, "" ) );
        viseme_map_.insert( std::pair<float, Ogre::String>( 0, "" ) );
        playTalkAnimation();
    }
}
//---------------------------------------------------------------------------

void TalkingHead::callbackTextForEmotion(const std_msgs::String::ConstPtr &msg)
{
    text_for_emotion_ = msg->data;
    if( word_map_.size() == 1 && viseme_map_.size() == 1 )
    {
        word_map_.clear();
        viseme_map_.clear();
    }
}
//---------------------------------------------------------------------------

void TalkingHead::callbackResetAnimation( const std_msgs::String::ConstPtr& msg)
{
    word_map_.clear();
    viseme_map_.clear();
}

void TalkingHead::callbackShowStream( const homer_robot_face::DisplayImage::ConstPtr& image_msg  )
{
    emit faceCleared();
    emit timerChanged( image_msg->time * 1000 );
}

void TalkingHead::callbackShowImage( const homer_robot_face::DisplayImageFile::ConstPtr& msg )
{
    emit faceCleared();
    emit timerChanged( msg->time * 1000);
}

void TalkingHead::callbackShowMatImage( const sensor_msgs::ImageConstPtr& msg )
{
    emit faceCleared();
    emit timerChanged( 1000 );
}

 void TalkingHead::clearFace()
 {
     setVisible( false );
 }

void TalkingHead::setTimer(int msec)
{
  redraw_timer_->start(msec);
}
