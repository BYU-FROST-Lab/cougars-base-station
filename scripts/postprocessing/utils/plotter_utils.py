from . import rosbags_converter as rc
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import chi2
from scipy.spatial.transform import Rotation as R

def get_dataframes(
        rosbags_dir:str,
        rosmsgs_dir:str,
        csv_dir:str,
        keywords:list[str],
        topics:list[str]=None,
        verbose:bool=True,
):
    print("converting rosbags")
    typestore = rc.generate_typestore(rosmsgs_dir)
    dataframes = rc.convert_rosbags(rosbags_dir, typestore, 
                                    keywords=keywords,verbose=verbose,
                                    topics=topics)
    rc.save_to_csv(dataframes, csv_dir, verbose=verbose)
    dataframes = rc.load_dataframes(csv_dir, keywords=keywords)
    return dataframes


def insert_timestamps(dataframes):
    for bag in dataframes.values():
        for df in bag.values():
            if "header.stamp.sec" in df.columns:
                df["timestamp"] = pd.to_datetime(
                    df["header.stamp.sec"] * 1_000_000_000 
                    + df["header.stamp.nanosec"], unit='ns'
                )

def get_topic(bag:dict[str,pd.DataFrame], topic_name:str, ns=None):
    if ns is None:
        for topic in bag.keys():
            if topic.endswith(topic_name):
                return bag[topic]
    else:
        if (ns+topic_name) in bag.keys(): return bag[ns+topic_name]
        else: return None
    return None



def plot_mahalanobis_ellipse(x, y, cov, confidence=0.95, ax=None, **kwargs):
    """
    Plot a 2D ellipse based on a Mahalanobis distance from a 2x2 covariance matrix.

    Parameters:
    - x, y: center of the ellipse
    - cov: 2x2 covariance matrix
    - confidence: confidence level for the ellipse (e.g. 0.95)
    - ax: matplotlib axis to plot on (optional)
    - kwargs: passed to matplotlib.patches.Ellipse

    written with ChatGPT
    """
    from matplotlib.patches import Ellipse

    # Ensure covariance is a 2x2 matrix
    cov = np.asarray(cov)
    if cov.shape != (2, 2):
        raise ValueError("Covariance matrix must be 2x2.")

    # Calculate the Mahalanobis radius for the given confidence
    chi2_val = chi2.ppf(confidence, df=2)
    vals, vecs = np.linalg.eigh(cov)  # eigenvalues and eigenvectors

    # Get width and height (2 * sqrt(eigenvalue * chi2_val))
    width, height = 2 * np.sqrt(vals * chi2_val)
    
    # Get angle in degrees of the first eigenvector
    angle = np.degrees(np.arctan2(*vecs[:, 1][::-1]))

    # Create ellipse
    ellipse = Ellipse((x, y), width, height, angle=angle, edgecolor='black',
                      facecolor='none', **kwargs)

    # Plot
    if ax is None:
        fig, ax = plt.subplots()
    ax.add_patch(ellipse)
    ax.set_aspect('equal')
    # ax.plot(x, y, 'ro')  # plot center
    ax.autoscale_view()
    return ax


def cov_from_str(cov_str:str):
    cov_list = [float(v) for v in cov_str[1:-1].split(" ") if v]
    size = int(np.sqrt(len(cov_list)))
    return np.array(cov_list).reshape((size,size))


def plot_pose_w_cov(
        pose_df,
        seconds_between_cov=2,
        confidence=.95,
        plot_direction_line=False,
        ax=None,
        **kwargs
):
    if ax is None:
        fig, ax = plt.subplots()

    x_dist = pose_df["pose.pose.position.x"].max() - pose_df["pose.pose.position.x"].min()
    y_dist = pose_df["pose.pose.position.y"].max() - pose_df["pose.pose.position.y"].min()
    dist = 0.04*max(x_dist, y_dist) # Used to calculate the length of orientation lines

    delta = pd.to_timedelta(seconds_between_cov, unit='s')
    last_time = pose_df["timestamp"].iloc[0]
    for idx, row in pose_df.iterrows():
    
        timestamp = row["timestamp"]
        if timestamp - last_time > delta:
            last_time = timestamp
            x = row["pose.pose.position.x"]
            y = row["pose.pose.position.y"]


            if plot_direction_line:
                R_ = R.from_quat([
                    row["pose.pose.orientation.x"],
                    row["pose.pose.orientation.y"],
                    row["pose.pose.orientation.z"],
                    row["pose.pose.orientation.w"]
                ])
                yaw = R_.as_euler('zyx', degrees=False)[0]
                plt.plot([x, x+dist*np.cos(yaw)],[y, y+dist*np.sin(yaw)], 'r-')
    
            cov = cov_from_str(row["pose.covariance"])
            xy_cov = cov[:2,:2]
            plot_mahalanobis_ellipse(x,y,xy_cov,confidence,ax)
            
    pose_x = pose_df["pose.pose.position.x"]
    pose_y = pose_df["pose.pose.position.y"]
    ax.plot(pose_x,pose_y, **kwargs)
    ax.plot(pose_x[0],pose_y[0],'o', **kwargs)

    ax.axis('equal')
    return ax


