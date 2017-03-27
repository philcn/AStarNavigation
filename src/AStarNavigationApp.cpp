#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/TextureFont.h"
#include "CinderImGui.h"
#include "sp/Grid.h"
#include "micropather.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace micropather;

class AStarNavigationApp : public App, public Graph {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void mouseMove( MouseEvent event ) override;
	void keyDown( KeyEvent event ) override;
	void update() override;
	void draw() override;
	
	SurfaceRef mMapSurface;
	SurfaceRef mStreetSurface;
	gl::TextureRef mMapTexture;
	gl::TextureRef mStreetTexture;
	
	// grid related
	// from https://github.com/simongeilfus/SpacePartitioning
	sp::Grid2<> mSpGrid;
	vector<vector<int>> mGrid;
	int mGridResolution = 600;
	int mXDim, mYDim;
	float mIncr;
	
	gl::BatchRef mSamplesBatch;
    bool mShowStreetLayer = false;
    bool mShowGrid = true;
	
	vec2 mStartPos;
	vec2 mEndPos;
	int mNumPosChosen = 0;
	
	// path related
	// from https://github.com/leethomason/MicroPather
	MicroPather *mPather;
	vector<vec2> mPath;
	BSpline2f mSpline;
	Path2d mPath2d;
	int mPathStep = 6;
	
	CameraPersp mCam;
	CameraPersp mTargetCam;
	
	// line drawing stuff
	// from https://github.com/paulhoux/Cinder-Samples/tree/master/GeometryShader
	gl::VboMeshRef mLineVboMesh;
	float mRadius = 5.0f;
	float mThickness = 5.0f;
	float mLimit = 0.75f;
	gl::GlslProgRef mLineShader;
	
	// text
	gl::TextureFontRef mTextureFont;
	Font mFont;
	
  protected:
	// coordinate conversion
  	vec2 indicesToPos( ivec2 indices );
	ivec2 posToIndices( vec2 pos );
	ivec2 stateToIndices( void *state );
	void *indicesToState( ivec2 indices );
	
	// protocol for A* library
	float LeastCostEstimate( void* stateStart, void* stateEnd ) override;
	void AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *adjacent ) override;
	void PrintStateInfo( void* state ) override;
	
	void initializeGrid();
	void findPath();
	void createSpline();
	void updateCamera();
	void updateLineVboMesh();
};

void AStarNavigationApp::setup()
{
	ui::initialize();
	ui::SetWindowFontScale( 2.0f );
	
	mMapSurface = Surface::create( loadImage( loadAsset( "manhattan.png" ) ) );
	mStreetSurface = Surface::create( mMapSurface->getWidth(), mMapSurface->getHeight(), false );

	// create street bitmap
	auto srcIter = mMapSurface->getIter();
	auto destIter = mStreetSurface->getIter();
	while( srcIter.line() ) {
		destIter.line();
		while( srcIter.pixel() ) {
			destIter.pixel();
			if( srcIter.r() == 69 && srcIter.g() == 69 && srcIter.b() == 69 ) {
				destIter.r() = 255;
				destIter.g() = 169;
				destIter.b() = 0;
			}
			else {
				destIter.r() = destIter.g() = destIter.b() = 0;
			}
		}
	}
	
	mMapTexture = gl::Texture::create( *mMapSurface );
	mStreetTexture = gl::Texture::create( *mStreetSurface );

	initializeGrid();
	
	mPather = new MicroPather( this );
	
	mCam = CameraPersp( mMapSurface->getWidth(), mMapSurface->getHeight(), 60 );
	mTargetCam = mCam;
	
	mLineShader = gl::GlslProg::create( loadAsset( "line.vert" ), loadAsset( "line.frag" ), loadAsset( "line.geom" ) );
	
	mFont = Font( "Courier New", 54 );
	mTextureFont = gl::TextureFont::create( mFont );
}

void AStarNavigationApp::initializeGrid()
{
	// derive grid dimensions
	mIncr = mStreetSurface->getWidth() / mGridResolution;
	mXDim = mStreetSurface->getWidth() / mIncr;
	mYDim = mStreetSurface->getHeight() / mIncr;
	
	// create acceleration structure for mosue picking
	mSpGrid = sp::Grid2<>( 3 );
	
	// create grid
	mGrid.clear();
	
	for( int j = 0; j < mYDim; j++ ) {
		mGrid.push_back( vector<int>() );
    	for( int i = 0; i < mXDim; i++ ) {
			mGrid[j].push_back( 0 );
			if( *mStreetSurface->getDataRed( indicesToPos( ivec2( i, j ) ) ) != 0 ) {
				mGrid[j][i] = 1;
			}
		}
	}
	
	// filter points according to connectivity and populate grid structures
	vector<vec2> streetSamples;
	for( int j = 1; j < mYDim - 1; j++ ) {
    	for( int i = 1; i < mXDim - 1; i++ ) {
			if( mGrid[j][i] == 1 ) {
				if( mGrid[j - 1][i - 1] == 1 ||
    					mGrid[j - 1][i    ] == 1 ||
    					mGrid[j - 1][i + 1] == 1 ||
    					mGrid[j    ][i - 1] == 1 ||
    					mGrid[j    ][i + 1] == 1 ||
    					mGrid[j + 1][i - 1] == 1 ||
    					mGrid[j + 1][i    ] == 1 ||
    					mGrid[j + 1][i + 1] == 1 ) {
        			vec2 pos = indicesToPos( ivec2( i, j ) );
    				mSpGrid.insert( pos );
    				streetSamples.push_back( pos );
				}
			}
		}
	}
	
	// batch for rendering grid points
	auto layout = geom::BufferLayout( { geom::AttribInfo( geom::POSITION, 2, 0, 0 ) } );
	auto vboMesh = gl::VboMesh::create( streetSamples.size(), GL_POINTS, { { layout, gl::Vbo::create( GL_ARRAY_BUFFER, streetSamples ) } } );
	mSamplesBatch = gl::Batch::create( vboMesh, gl::getStockShader( gl::ShaderDef().color() ) );
}

void AStarNavigationApp::keyDown( KeyEvent event )
{
	if( event.getChar() == ' ' ) {
    	// press space bar to initiate path finding
		if( mPath.size() > 0 ) {
			mNumPosChosen = 0;
			mPather->Reset();
			mPath.clear();
			mPath2d.clear();
			mSpline = BSpline2f();
			mTargetCam = CameraPersp( mMapSurface->getWidth(), mMapSurface->getHeight(), 60 );
		}
		// or clear the current query
		else if( mNumPosChosen == 2 ) {
			findPath();
		}
	}
}

void AStarNavigationApp::mouseDown( MouseEvent event )
{
	// convert from window space to map space
	vec2 mousePos = event.getPos();
	vec2 mapSpacePos = mousePos * float( mMapSurface->getWidth() ) / float( getWindowWidth() );
	sp::Grid2<>::Node *node = mSpGrid.nearestNeighborSearch( mapSpacePos );
	auto gridPos = node->getPosition();
	
	if( mNumPosChosen == 0 ) {
    	mStartPos = gridPos;
		mNumPosChosen++;
	}
	else if( mNumPosChosen == 1 ) {
		mEndPos = gridPos;
		mNumPosChosen++;
	}
	else {
		mNumPosChosen = 0;
    	mPather->Reset();
    	mPath.clear();
    	mPath2d.clear();
		mSpline = BSpline2f();
    	mTargetCam = CameraPersp( mMapSurface->getWidth(), mMapSurface->getHeight(), 60 );
	}
}

void AStarNavigationApp::mouseMove( MouseEvent event )
{
	// hold option for interactive mode
	if( ! event.isAltDown() )
		return;
	
	vec2 mousePos = event.getPos();
	vec2 mapSpacePos = mousePos * float( mMapSurface->getWidth() ) / float( getWindowWidth() );
	sp::Grid2<>::Node *node = mSpGrid.nearestNeighborSearch( mapSpacePos );
	auto gridPos = node->getPosition();
	
	if( mNumPosChosen > 0 ) {
		mEndPos = gridPos;
		mNumPosChosen = 2;
		
    	mPather->Reset();
    	mPath.clear();
    	mPath2d.clear();
		mSpline = BSpline2f();
		findPath();
	}
}

vec2 AStarNavigationApp::indicesToPos( ivec2 indices )
{
	return vec2( indices ) * mIncr;
}

ivec2 AStarNavigationApp::posToIndices( vec2 pos )
{
	return ivec2( pos / mIncr );
}

ivec2 AStarNavigationApp::AStarNavigationApp::stateToIndices( void *state )
{
	intptr_t index = (intptr_t)state;
	ivec2 res;
	res.y = index / mXDim;
	res.x = index - res.y * mXDim;
	return res;
}

void* AStarNavigationApp::indicesToState( ivec2 indices )
{
	return (void *)(intptr_t)( indices.y * mXDim + indices.x );
}

float AStarNavigationApp::LeastCostEstimate( void* stateStart, void* stateEnd )
{
	ivec2 idxStart = stateToIndices( stateStart );
	ivec2 idxEnd = stateToIndices( stateEnd );
	
	int dx = idxStart.x - idxEnd.x;
	int dy = idxStart.y - idxEnd.y;
	
	return (float)sqrt( (double)( dx * dx ) + (double)( dy * dy ) );
}

void AStarNavigationApp::AdjacentCost( void* state, MP_VECTOR< micropather::StateCost > *neighbors )
{
	int x = stateToIndices( state ).x;
	int y = stateToIndices( state ).y;
	
	const int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
	const int dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
	const float cost[8] = { 1.0f, 1.41f, 1.0f, 1.41f, 1.0f, 1.41f, 1.0f, 1.41f };
	
	for( int i = 0; i < 8; ++i ) {
		int nx = x + dx[i];
		int ny = y + dy[i];
		
		if( nx < 0 || nx >= mXDim || ny < 0 || ny >= mYDim )
			continue;
		
		if( mGrid[ny][nx] == 1 ) {
			StateCost nodeCost = { indicesToState( ivec2( nx, ny ) ), cost[i] };
			neighbors->push_back( nodeCost );
		}
	}
}

void AStarNavigationApp::PrintStateInfo( void* state )
{
	// no-op
}

void AStarNavigationApp::findPath()
{
	if( mNumPosChosen != 2 )
		return;
	
	// don't calculate for the same route
	if( mPath.size() > 0 )
		return;

    MPVector<void*> path;
    float totalCost = 0;
	void *startState = indicesToState( posToIndices( mStartPos ) );
	void *endState = indicesToState( posToIndices( mEndPos ) );
	
    int result = mPather->Solve( startState, endState, &path, &totalCost );
	
	if( result == MicroPather::SOLVED ) {
		for( int i = 0; i < path.size(); ++i )
			mPath.push_back( indicesToPos( stateToIndices( path[i] ) ) );
		
		createSpline();
		updateCamera();
	}
}

void AStarNavigationApp::createSpline()
{
	int step = mPathStep;
	if( mPath.size() < 3 * step )
		step = 1;
	
	vector<vec2> samplePoints;
	for( int i = 0; i < mPath.size(); i += step ) {
		samplePoints.push_back( mPath[i] );
	}
	
	if( samplePoints.size() > 3 ) {
		// create a smoothed B-spline
		mSpline = BSpline2f( samplePoints, 3, false, true );
		mPath2d = Path2d( mSpline );
		
		// create VboMesh for thick line rendering
		updateLineVboMesh();
	}
}

void AStarNavigationApp::updateCamera()
{
	// aim at the center of the path
	vec2 center = ( mStartPos + mEndPos ) * 0.5f;
	center.y = mMapSurface->getHeight() - center.y;
	
	mTargetCam = CameraPersp( mMapSurface->getWidth(), mMapSurface->getHeight(), 60 );
	float defaultZ = mTargetCam.getEyePoint().z;
	
	// TODO: calculate correct Z to frame the new route
	vec3 eyePoint( center, defaultZ - 1000 );
	vec3 target( center, 0 );
	mTargetCam.lookAt( eyePoint, target );
	
	quat defaultOrientation = mTargetCam.getOrientation();
	mTargetCam.setOrientation( glm::rotate( defaultOrientation, glm::radians( 5.0f ), vec3( 0, 0, 1 ) ) );
}

void AStarNavigationApp::update()
{
	getWindow()->setTitle( to_string( int( getAverageFps() ) ) );
	
	ui::Checkbox( "Show street layer", &mShowStreetLayer );
	ui::Checkbox( "Show grid", &mShowGrid );
	
	ui::SliderInt( "Grid resolution", &mGridResolution, 300, 1200 );
	
	if( ui::Button( "Regenerate grid" ) )
		initializeGrid();
	
	if( ui::SliderInt( "Path step", &mPathStep, 1, 8 ) )
		createSpline();
	
	if( ui::Button( "Find path" ) )
		findPath();
}

void AStarNavigationApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	
	gl::ScopedMatrices scopedMtx;
//	gl::setMatricesWindow( mMapSurface->getWidth(), mMapSurface->getHeight() );

	// animate camera lerp
	mCam.setEyePoint( glm::lerp( mCam.getEyePoint(), mTargetCam.getEyePoint(), 0.025f ) );
	mCam.setOrientation( glm::lerp( mCam.getOrientation(), mTargetCam.getOrientation(), 0.025f ) );
	
	gl::setMatrices( mCam );
	gl::setViewMatrix( gl::getViewMatrix() * glm::scale( vec3( 1, -1, 1 ) ) * glm::translate( vec3( 0, (float) - mMapSurface->getHeight(), 0 ) ) );
	
	gl::draw( mMapTexture );
	
	if( mShowStreetLayer ) {
    	gl::ScopedBlendAdditive scopedBlend;
    	gl::ScopedColor scopedColor( ColorAf( 1, 1, 1, 0.7 ) );
    	gl::draw( mStreetTexture );
	}
	
	if( mShowGrid ) {
    	gl::ScopedBlendAdditive scopedBlend;
    	gl::ScopedColor scopedColor( ColorAf( 0.5, 0, 0, 1 ) );
		gl::pointSize( 2 );
    	mSamplesBatch->draw();
	}
	
	if( mNumPosChosen > 0 ) {
    	gl::ScopedBlendAdditive scopedBlend;
    	gl::ScopedColor scopedColor( ColorAf( 0, 1, 1, 1 ) );
    	gl::drawStrokedCircle( mStartPos, 18, 4, 4 );
		
    	gl::ScopedColor scopedColorCircle( ColorAf( 0, 1, 1, 0.3 ) );
    	gl::drawSolidCircle( mStartPos, 22 );
		
    	gl::ScopedColor scopedColorShadow( ColorAf( 0.3, 0.3, 0.3, 0.7 ) );
		mTextureFont->drawString( "Starting point", mStartPos + vec2( -14, -36 ) );
		
    	gl::ScopedColor scopedColorText( ColorAf( 1, 1, 1, 1 ) );
		mTextureFont->drawString( "Starting point", mStartPos + vec2( -20, -40 ) );
	}
	if( mNumPosChosen > 1 ) {
    	gl::ScopedBlendAdditive scopedBlend;
    	gl::ScopedColor scopedColor( ColorAf( 0.1, 1, 0.3, 1 ) );
    	gl::drawStrokedCircle( mEndPos, 18, 4, 4 );
		
    	gl::ScopedColor scopedColorCircle( ColorAf( 0, 1, 1, 0.3 ) );
    	gl::drawSolidCircle( mEndPos, 22 );
		
    	gl::ScopedColor scopedColorShadow( ColorAf( 0.3, 0.3, 0.3, 0.7 ) );
		mTextureFont->drawString( "Destination", mEndPos + vec2( -14, -36 ) );
		
    	gl::ScopedColor scopedColorText( ColorAf( 1, 1, 1, 1 ) );
		mTextureFont->drawString( "Destination", mEndPos + vec2( -20, -40 ) );
	}
	
	if( mPath.size() > 0 ) {
    	gl::ScopedBlendAdditive scopedBlend;
    	gl::ScopedColor scopedColor( ColorAf( 1, 1, 1, 1 ) );
        gl::ScopedGlslProg shader( mLineShader );
		mLineShader->uniform( "WIN_SCALE", vec2( getWindowSize() ) ); // casting to vec2 is mandatory!
		mLineShader->uniform( "MITER_LIMIT", mLimit );
		mLineShader->uniform( "THICKNESS", mThickness );
		
		gl::draw( mLineVboMesh );
	}
}

void AStarNavigationApp::updateLineVboMesh()
{
	mLineVboMesh.reset();
	
	vector<vec2> mPoints;
	int numSegments = 80;
	for( int i = 0; i <= numSegments; i++ ) {
		vec2 pos;
		mSpline.get( float( i ) / numSegments, &pos );
		mPoints.push_back( pos );
	}
	
	if( mPoints.size() > 1 ) {
		// create a new vector that can contain 3D vertices
		std::vector<vec3> vertices;
		
		// to improve performance, make room for the vertices + 2 adjacency vertices
		vertices.reserve( mPoints.size() + 2 );
		
		// first, add an adjacency vertex at the beginning
		vertices.push_back( 2.0f * vec3( mPoints[0], 0 ) - vec3( mPoints[1], 0 ) );
		
		// next, add all 2D points as 3D vertices
		std::vector<vec2>::iterator itr;
		for( itr = mPoints.begin(); itr != mPoints.end(); ++itr )
			vertices.push_back( vec3( *itr, 0 ) );
		
		// next, add an adjacency vertex at the end
		size_t n = mPoints.size();
		vertices.push_back( 2.0f * vec3( mPoints[n - 1], 0 ) - vec3( mPoints[n - 2], 0 ) );
		
		// now that we have a list of vertices, create the index buffer
		n = vertices.size() - 2;
		std::vector<uint16_t> indices;
		indices.reserve( n * 4 );
		
		for( size_t i = 1; i < vertices.size() - 2; ++i ) {
			indices.push_back( i - 1 );
			indices.push_back( i );
			indices.push_back( i + 1 );
			indices.push_back( i + 2 );
		}
		
		// finally, create the mesh
		gl::VboMesh::Layout layout;
		layout.attrib( geom::POSITION, 3 );
		
		mLineVboMesh = gl::VboMesh::create( vertices.size(), GL_LINES_ADJACENCY_EXT, { layout }, indices.size() );
		mLineVboMesh->bufferAttrib( geom::POSITION, vertices.size() * sizeof( vec3 ), vertices.data() );
		mLineVboMesh->bufferIndices( indices.size() * sizeof( uint16_t ), indices.data() );
	}
}

CINDER_APP( AStarNavigationApp, RendererGl( RendererGl::Options().msaa( 16 ) ), [] ( App::Settings *settings ) {
	settings->setWindowSize( 1080, 950 );
	settings->setHighDensityDisplayEnabled();
} )
