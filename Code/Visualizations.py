import matplotlib.pyplot as plt
import numpy as np
from RK_Driver import dcm_from_q

# Make a 2D plot of the results for the angular velocity
# Inputs:
# - Time array (time[ N ]) [sec]
# - Angular rate array (om_arr[ N ][ 3 ]) [rad/s]
# - fig-name:
# -- "0" if no figure is to be saved
# -- string which is the fig name otherwise
def plot_2D( time , om_arr , fig_name ):

    # Plot the results
    fig, ax = plt.subplots()

    ax.plot( time , np.transpose( om_arr )[ 0 ] , linestyle = "solid" , label = r"\omega_x" )
    ax.plot( time , np.transpose( om_arr )[ 1 ] , linestyle = "solid" , label = r"\omega_y" )
    ax.plot( time , np.transpose( om_arr )[ 2 ] , linestyle = "solid" , label = r"\omega_z" )

    plt.xlabel( "Time (s)" )
    plt.ylabel( "Angular Rate [rad/s]" )
    plt.title( "Intermediate Axis Plot" )

    plt.grid()

    if fig_name != "0":
        fig.savefig( fig_name + ".png" , format = "png" )

    plt.show( )

# Make a 3D animation of the results for the angular velocity
# Inputs:
# - Time array (time[ N ]) [sec]
# - Angular rate array (om_arr[ N ][ 3 ]) [rad/s]
def animate_3D( time , om_arr ):

    ax = plt.figure( ).add_subplot( projection = "3d" )    
    
    for i in range( 0 , len( time ) ):
        plt.cla( )
        ax.quiver( 0 , 0 , 0 , om_arr[ i ][ 0 ] , om_arr[ i ][ 1 ] , om_arr[ i ][ 2 ] , normalize = True )

        ax.set_xlim3d( -1.2 , 1.2 )
        ax.set_ylim3d( -1.2 , 1.2 )
        ax.set_zlim3d( -1.2 , 1.2 )

        plt.pause( 0.1 )

# Make a 3D animation of 3 basis vectors (dreibein) as the body-fixed axes
# Inputs:
# - Time array (time[ N ]) [sec]
# - Unit quaternion array (q_arr[ N ][ 4 ]) [-]
def animate_dreibein( time , q_arr ):

    # Define the initial vectors for [ 1 , 0 , 0 , 0 ]
    e_1 = [ 1.0 , 0.0 , 0.0 ]
    e_2 = [ 0.0 , 1.0 , 0.0 ]
    e_3 = [ 0.0 , 0.0 , 1.0 ]
    # These forma triad (dreibein) of vectors { e_1 , e_2 , e_3 } which are orthonormal
    # They will be rotated to visualize a body's motion

    # Initialize 3 vectors which will hold the rotated dreibein for each step
    v_1 = np.zeros( 3 )
    v_2 = np.zeros( 3 )
    v_3 = np.zeros( 3 )

    ax = plt.figure( ).add_subplot( projection = "3d" )    
    
    for i in range( 0 , len( time ) ):

        # Compute the DCM from the current quaternion
        dcm_q = dcm_from_q( q_arr[ i ] )

        v_1 = np.matmul( dcm_q , e_1 )
        v_2 = np.matmul( dcm_q , e_2 )
        v_3 = np.matmul( dcm_q , e_3 )

        plt.cla( )
        ax.quiver( 0 , 0 , 0 , v_1[ 0 ] , v_1[ 1 ] , v_1[ 2 ] , normalize = True , color = "red" )
        ax.quiver( 0 , 0 , 0 , v_2[ 0 ] , v_2[ 1 ] , v_2[ 2 ] , normalize = True , color = "green" )
        ax.quiver( 0 , 0 , 0 , v_3[ 0 ] , v_3[ 1 ] , v_3[ 2 ] , normalize = True , color = "blue" )

        ax.set_xlim3d( -1.2 , 1.2 )
        ax.set_ylim3d( -1.2 , 1.2 )
        ax.set_zlim3d( -1.2 , 1.2 )

        plt.pause( 0.01 )
