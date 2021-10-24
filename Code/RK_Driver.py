# Runge-Kutta Driver for quaternions-described object orientation

# Import other packages used
import numpy as np

# Import constants and parameters used
from Parameters_and_Constants import pi, moi

# Dual Matrix to a vector
# Inputs:
# - 3D array ( vec[ 3 ] )
# Outputs:
# - 3x3 matrix ( dvec ) which is its dual for vector product ( vec x a = dvec . a )
def dual_matrix( vec ):

    dvec = [ [ 0.0 , - vec[ 2 ] , vec[ 1 ] ] ,
             [ vec[ 2 ] , 0.0 , - vec[ 0 ] ] ,
             [ - vec[ 1 ] , vec[ 0 ] , 0.0 ] ]

    return dvec

# Dual Matrix to a quaternion
# Inputs:
# - 4D array for the quaternion ( qu[ 4 ] )
# Outputs:
# - 4x3 matrix ( dquat ) which is its dual for quaternion left multiplication with a vector
def q_mat( qu ):

    dquat = [ [ - qu[ 1 ] , - qu[ 2 ] , - qu[ 3 ] ] ,
              [ qu[ 0 ] , - qu[ 3 ] , qu[ 2 ] ] ,
              [ qu[ 3 ] , qu[ 0 ] , - qu[ 1 ] ] ,
              [ - qu[ 2 ] , qu[ 1 ] , qu[ 0 ] ] ]

    return dquat

# Directive Cosine Matrix (DCM) from a Quaternion
# Inputs:
# - 4D array for the unit quaternion ( qu[ 4 ] )
# Outputs:
# - Directive Cosine Matrix (DCM) which is a 3x3 SO(3) rep dcm[ 3 ][ 3 ]
# NOTE: The quaternion is renormed for each case (to make it unit even if it's not)
def dcm_from_q( qu ):

    # If the length of the quaternion was wrong - return unit matrix and print the error
    if len( qu ) != 4:
        print( "Wrong quaternion length, len( qu ) == 4 is required!" )
        print( "Returning unit DCM = diag( 1 , 1 , 1 )" )
        
        dcm = [ [ 1.0 , 0.0 , 0.0 ] ,
                [ 0.0 , 1.0 , 0.0 ] ,
                [ 0.0 , 0.0 , 1.0 ] ]
        return dcm
    
    # Find the norm and renorm the Quaternion for each case
    q_norm = np.sqrt( np.dot( qu , qu ) )
    for i in range( 0 , 4 ):
        qu[ i ] /= q_norm
    
    # Compute squares of quaternion and then compute the dcm
    q0s = qu[ 0 ]*qu[ 0 ]
    q1s = qu[ 1 ]*qu[ 1 ]
    q2s = qu[ 2 ]*qu[ 2 ]
    q3s = qu[ 3 ]*qu[ 3 ]

    dcm = [ [ q0s + q1s - q2s - q3s , 2.0*( qu[ 1 ]*qu[ 2 ] + qu[ 0 ]*qu[ 3 ] ) , 2.0*( qu[ 1 ]*qu[ 3 ] - qu[ 0 ]*qu[ 2 ] ) ] ,
            [ 2.0*( qu[ 2 ]*qu[ 1 ] - qu[ 0 ]*qu[ 3 ] ) , q0s - q1s + q2s - q3s , 2.0*( qu[ 2 ]*qu[ 3 ] + qu[ 0 ]*qu[ 1 ] ) ] ,
            [ 2.0*( qu[ 3 ]*qu[ 1 ] + qu[ 0 ]*qu[ 2 ] ) , 2.0*( qu[ 3 ]*qu[ 2 ] - qu[ 0 ]*qu[ 1 ] ) , q0s - q1s - q2s + q3s ] ]

    return dcm

# Torque function
# Inputs:
# - time from some arbitrary epoch time [sec]
# Outputs:
# - torque vector as a 3D array [N*m]
# NOTE: CURRENTLY RETURNING 0 VECTOR -> NO EXTERNAL TORQUE CONSIDERED
def torque_func( time ):

    torque = [ 0.0 , 0.0 , 0.0 ]

    return torque

# Right-Hand-Side (RHS) of the body attitude state
# Inputs:
# - Attitude State ( state[ 2 ] = [ qu[ 4 ] , om[ 3 ] ] ) consisting of:
# -- quaternion attitude ( qu[ 4 ] ) [-]
# -- angular velocity ( om[ 3 ] ) [rad/s] in Body Frame
# - External Torques ( torque[ 3 ] ) [N*m] in Inertial Frame
# Outputs:
# - Right-Hand-Side of state's derivative ( rhs_state[ 2 ] = [ rhs_qu[ 4 ] , rhs_om[ 3 ] ] ) consisting of:
# -- quaternion RHS ( rhs_qu[ 4 ] ) [1/s]
# -- angular velocity RHS ( rhs_om[ 3 ] ) [rad/s^2]
def rhs_quaternion( state , torque ):

    dual_om = dual_matrix( state[ 1 ] ) # get angular velocity dual matrix
    dual_quat = q_mat( state[ 0 ] ) # get quaternion dual matrix

    # Get RHS for the angular rate part from angular momentum conservation equation
    rhs_om = - np.matmul( dual_om , np.matmul( moi , state[ 1 ] ) ) + np.array( torque )
    rhs_om = np.matmul( np.linalg.inv( moi ) , rhs_om )

    # Get RHS for the quaternion part from quaternion derivative equation
    rhs_qu = 0.5*np.matmul( dual_quat , state[ 1 ] )

    rhs_state = [ rhs_qu , rhs_om ]

    return rhs_state

# Perform a Runge-Kutta Step of the attitude state
# Inputs:
# - Starting Attitude State ( state_in[ 2 ] = [ qu[ 4 ] , om[ 3 ] ] ) consisting of:
# -- quaternion attitude ( qu[ 4 ] ) [-]
# -- angular velocity ( om[ 3 ] ) [rad/s] in Body Frame
# - Initial Time [sec] (measured from some initial epoch)
# - Time Step (between input and output attitude states) [sec]
# Outputs:
# - Resulting Attitude State ( state_out[ 2 ] = [ qu[ 4 ] , om[ 3 ] ] ) consisting of:
# -- quaternion attitude ( qu[ 4 ] ) [-]
# -- angular velocity ( om[ 3 ] ) [rad/s] in Body Frame
def rk_step( state_in , t_i , dt ):

    # Initialize Runge-Kutta coefficients arrays for the quaternion and angular velocity components
    rk_qu = np.zeros( ( 4 , 4 ) ) # Quaternion RK coefficients 4 x <dimension>
    rk_om = np.zeros( ( 4 , 3 ) ) # Angular rate RK coefficients 4 x <dimension>
    # Initialize intermediate state to be populated for intermediate computations
    inter_state = [ np.zeros( 4 ) , np.zeros( 3 ) ]

    # Find external torque at initial step
    torque = torque_func( t_i )

    # Populate RK constants values at the first step ( k1 )
    rhs_state = rhs_quaternion( state_in , torque )
    rk_qu[ 0 ] = rhs_state[ 0 ]*dt
    rk_om[ 0 ] = rhs_state[ 1 ]*dt
    # Find intermediate state ( t + dt/2 , x + k1/2 ) and corresponding torque in preparation for next step
    inter_state[ 0 ] = state_in[ 0 ] + rk_qu[ 0 ]/2.0
    inter_state[ 1 ] = state_in[ 1 ] + rk_om[ 0 ]/2.0
    torque = torque_func( t_i + dt/2.0 )

    # Populate RK constants values at the second step ( k2 )
    rhs_state = rhs_quaternion( inter_state , torque )
    rk_qu[ 1 ] = rhs_state[ 0 ]*dt
    rk_om[ 1 ] = rhs_state[ 1 ]*dt
    # Find intermediate state ( t + dt/2 , x + k2/2 ), corresponding torque is the same (same time)
    inter_state[ 0 ] = state_in[ 0 ] + rk_qu[ 1 ]/2.0
    inter_state[ 1 ] = state_in[ 1 ] + rk_om[ 1 ]/2.0

    # Populate RK constants values at the third step ( k3 )
    rhs_state = rhs_quaternion( inter_state , torque )
    rk_qu[ 2 ] = rhs_state[ 0 ]*dt
    rk_om[ 2 ] = rhs_state[ 1 ]*dt
    # Find intermediate state ( t + dt , x + k3 ) and corresponding torque in preparation for the last step
    inter_state[ 0 ] = state_in[ 0 ] + rk_qu[ 2 ]
    inter_state[ 1 ] = state_in[ 1 ] + rk_om[ 2 ]
    torque = torque_func( t_i + dt )

    # Populate RK constants values at the last (forth) step ( k4 )
    rhs_state = rhs_quaternion( inter_state , torque )
    rk_qu[ 3 ] = rhs_state[ 0 ]*dt
    rk_om[ 3 ] = rhs_state[ 1 ]*dt

    # Compute the state at t_i + dt based on the RK values computed - populate this in inter_state
    inter_state[ 0 ] = state_in[ 0 ] + ( rk_qu[ 0 ] + 2.0*rk_qu[ 1 ] + 2.0*rk_qu[ 2 ] + rk_qu[ 3 ] )/6.0
    inter_state[ 1 ] = state_in[ 1 ] + ( rk_om[ 0 ] + 2.0*rk_om[ 1 ] + 2.0*rk_om[ 2 ] + rk_om[ 3 ] )/6.0

    # Out quaternions must be unit to describe valid rotation!
    # Compute the norm of the quaternion part and if different from 1, renorm it accordingly!
    q_norm = np.sqrt( np.dot( state_in[ 0 ] , state_in[ 0 ] ) )

    # The only problematic point is if q_norm == 0 -> this is physically impossible (only if unphysical initial conditions were given)
    if q_norm == 0.0:
        print( "ERROR: Quaternion came up with 0 norm, invalid attitude specified!" )
        inter_state = [ np.zeros( 4 ) , np.zeros( 3 ) ]
    else:
        inter_state[ 0 ] /= q_norm

    return inter_state
