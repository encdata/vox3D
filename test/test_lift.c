#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "box3d/voxel.h"

#include <float.h>
#include <math.h>

static int ShapeLiftDefaults( void )
{
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	ENSURE( shapeDef.enableLift == false );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 1.0f };
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	ENSURE( b3Shape_IsLiftEnabled( shapeId ) == false );

	b3Shape_EnableLift( shapeId, true );
	ENSURE( b3Shape_IsLiftEnabled( shapeId ) == true );

	b3Shape_EnableLift( shapeId, false );
	ENSURE( b3Shape_IsLiftEnabled( shapeId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int HullLiftAndDragTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero; // zero gravity
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// 1. Aerodynamic wing plate tilted upward (pitch angle 10 degrees)
	float pitchRad = 10.0f * B3_PI / 180.0f;
	b3Quat pitch = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );

	b3BodyDef bodyDefA = b3DefaultBodyDef();
	bodyDefA.type = b3_dynamicBody;
	bodyDefA.motionLocks.angularX = true;
	bodyDefA.motionLocks.angularY = true;
	bodyDefA.motionLocks.angularZ = true;
	bodyDefA.position = ( b3Pos ){ 0.0f, 10.0f, 0.0f };
	bodyDefA.rotation = pitch;
	bodyDefA.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, -20.0f }; // moving forward in -Z
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDefA );

	b3BoxHull plate = b3MakeBoxHull( 2.0f, 0.02f, 1.0f );
	b3ShapeDef shapeDefA = b3DefaultShapeDef();
	shapeDefA.density = 100.0f;
	shapeDefA.enableLift = true;
	b3CreateHullShape( bodyA, &shapeDefA, &plate.base );

	// 2. Control plate with lift disabled
	b3BodyDef bodyDefB = bodyDefA;
	bodyDefB.position = ( b3Pos ){ 10.0f, 10.0f, 0.0f };
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDefB );

	b3ShapeDef shapeDefB = b3DefaultShapeDef();
	shapeDefB.density = 100.0f;
	shapeDefB.enableLift = false;
	b3CreateHullShape( bodyB, &shapeDefB, &plate.base );

	// Simulate 1 step
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 velA = b3Body_GetLinearVelocity( bodyA );
	b3Vec3 velB = b3Body_GetLinearVelocity( bodyB );

	// Body A should have gained upward velocity (Lift > 0) and slowed down in forward speed (Drag > 0)
	ENSURE( velA.y > 0.1f );
	ENSURE( velA.z > -20.0f ); // slowed down from -20

	// Body B with lift disabled should remain at constant velocity in vacuum
	ENSURE_SMALL( velB.y, 1e-4f );
	ENSURE_SMALL( velB.z - ( -20.0f ), 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int VoxelLiftAndDragTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.wind = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// 1. Tilted voxel plate moving forward
	float pitchRad = 12.0f * B3_PI / 180.0f;
	b3Quat pitch = b3MakeQuatFromAxisAngle( b3Vec3_axisX, pitchRad );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.position = ( b3Pos ){ 0.0f, 15.0f, 0.0f };
	bodyDef.rotation = pitch;
	bodyDef.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, -25.0f }; // forward in -Z
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Build a 9x1x5 flat voxel plate
	b3Vec3i cells[45];
	int count = 0;
	for ( int x = -4; x <= 4; ++x )
	{
		for ( int z = -2; z <= 2; ++z )
		{
			cells[count++] = ( b3Vec3i ){ x, 0, z };
		}
	}

	b3VoxelData* voxels = b3CreateVoxelData( cells, count, 0.5f );
	ENSURE( voxels != NULL );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20.0f;
	shapeDef.enableLift = true;
	b3CreateVoxelShape( bodyId, &shapeDef, voxels );

	// Step simulation 1 step
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );

	// Voxel plate must experience upward lift (vel.y > 0) and drag (vel.z slowed from -25)
	ENSURE( vel.y > 0.1f );
	ENSURE( vel.z > -25.0f );

	b3DestroyWorld( worldId );
	return 0;
}

static int WorldWindTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.wind = ( b3Vec3 ){ 15.0f, 0.0f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3Vec3 wind = b3World_GetWind( worldId );
	ENSURE_SMALL( wind.x - 15.0f, 1e-5f );
	ENSURE_SMALL( wind.y, 1e-5f );
	ENSURE_SMALL( wind.z, 1e-5f );

	// Change wind at runtime
	b3World_SetWind( worldId, ( b3Vec3 ){ 25.0f, 5.0f, -2.0f } );
	wind = b3World_GetWind( worldId );
	ENSURE_SMALL( wind.x - 25.0f, 1e-5f );
	ENSURE_SMALL( wind.y - 5.0f, 1e-5f );
	ENSURE_SMALL( wind.z - ( -2.0f ), 1e-5f );

	// Create a resting body with enableLift = true
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableLift = true;
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3World_SetWind( worldId, ( b3Vec3 ){ 20.0f, 0.0f, 0.0f } );

	// Step simulation
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );
	// Ambient wind should accelerate the sphere in +X direction
	ENSURE( vel.x > 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

static int ShapeApplyWindExplicitTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull plate = b3MakeBoxHull( 1.0f, 0.02f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableLift = false; // test manual ApplyWind
	b3ShapeId shapeId = b3CreateHullShape( bodyId, &shapeDef, &plate.base );

	// Apply wind explicitly
	b3Shape_ApplyWind( shapeId, ( b3Vec3 ){ 0.0f, 20.0f, 0.0f }, 1.0f, 1.0f, 50.0f, true );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );
	ENSURE( vel.y > 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

static int AirfoilPolarLiftAndDragTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// 1. Box with Cambered Airfoil flying with 0 pitch
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.position = ( b3Pos ){ 0.0f, 10.0f, 0.0f };
	bodyDef.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, -20.0f }; // forward in -Z
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeBoxHull( 2.0f, 0.2f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20.0f;
	shapeDef.enableLift = true;
	shapeDef.airfoil = b3MakeCamberedAirfoil( ( b3Vec3 ){ 0.0f, 0.0f, -1.0f }, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f }, 2.0f, 6.0f );
	b3ShapeId shapeId = b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Airfoil retrieved = b3Shape_GetAirfoil( shapeId );
	ENSURE( retrieved.type == b3_airfoilCambered );
	ENSURE_SMALL( retrieved.zeroLiftAoA - ( -3.5f * B3_PI / 180.0f ), 1e-4f );

	// Cambered airfoil generates positive lift even at 0 deg angle of attack due to camber
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );
	ENSURE( vel.y > 0.01f );
	ENSURE( vel.z > -20.0f );

	// Test runtime change to Symmetric Airfoil
	b3Airfoil symm = b3MakeSymmetricAirfoil( ( b3Vec3 ){ 0.0f, 0.0f, -1.0f }, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f }, 2.0f, 6.0f );
	b3Shape_SetAirfoil( shapeId, &symm );
	retrieved = b3Shape_GetAirfoil( shapeId );
	ENSURE( retrieved.type == b3_airfoilSymmetric );

	b3DestroyWorld( worldId );
	return 0;
}

static int ProxyHullLiftTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 10.0f, 0.0f };
	bodyDef.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, -20.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Collision shape is a sphere, but aerodynamic proxy is a flat plate hull
	b3BoxHull proxyPlate = b3MakeBoxHull( 2.0f, 0.02f, 1.0f );
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20.0f;
	shapeDef.enableLift = true;
	shapeDef.airfoil = b3MakeProxyHullAirfoil( &proxyPlate.base );
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );
	// Drag from proxy plate should decelerate body
	ENSURE( vel.z > -20.0f );

	b3DestroyWorld( worldId );
	return 0;
}

static int WindAutoWakeTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.wind = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 5.0f, 0.0f };
	bodyDef.enableSleep = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull plate = b3MakeBoxHull( 1.0f, 0.02f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 5.0f;
	shapeDef.enableLift = true;
	b3CreateHullShape( bodyId, &shapeDef, &plate.base );

	// Step simulation until body goes to sleep
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( b3Body_IsAwake( bodyId ) == false );

	// Activate wind
	b3World_SetWind( worldId, ( b3Vec3 ){ 25.0f, 0.0f, 0.0f } );

	// Next step must auto-wake the body and apply wind force
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	ENSURE( b3Body_IsAwake( bodyId ) == true );
	b3Vec3 vel = b3Body_GetLinearVelocity( bodyId );
	ENSURE( vel.x > 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

int LiftTest( void )
{
	RUN_SUBTEST( ShapeLiftDefaults );
	RUN_SUBTEST( HullLiftAndDragTest );
	RUN_SUBTEST( VoxelLiftAndDragTest );
	RUN_SUBTEST( WorldWindTest );
	RUN_SUBTEST( ShapeApplyWindExplicitTest );
	RUN_SUBTEST( AirfoilPolarLiftAndDragTest );
	RUN_SUBTEST( ProxyHullLiftTest );
	RUN_SUBTEST( WindAutoWakeTest );

	return 0;
}

