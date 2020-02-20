"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016 Alex Bewley alex@dynamicdetection.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import numpy as np
from sklearn.metrics.pairwise import euclidean_distances
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter


class KalmanTracker(object):
    """
    This class represents the internal state of individual tracked objects observed as bbox.
    """
    count = 0

    def __init__(self, coords: np.ndarray):
        """
        Initialises a tracker using initial coordinates.

        @:param coords: initial 2D coordinates (x, y) of the object to be tracked
        """
        # define constant velocity model
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])

        self.kf.P[3:, 3:] *= 1000.  # give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10.
        self.kf.Q[3:, 3:] *= 0.01

        self.kf.x[:3] = coords.reshape((3, 1))
        self.time_since_update = 0
        self.id = KalmanTracker.count
        KalmanTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def update(self, coords: np.ndarray):
        """
        Updates the state vector with observed coordinates.

        @:param coords: updated 2D coordinates (x, y) of the object to be tracked
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(coords.reshape((3, 1)))

    def predict(self) -> np.ndarray:
        """
        Advances the state vector and returns the predicted coordinate estimate.
        """
        self.kf.predict()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(self.kf.x)
        return self.history[-1][:3].reshape((1, 3))

    def get_state(self) -> np.ndarray:
        """
        Returns the current state estimate.
        """
        return self.kf.x.reshape((1, 6))


def associate_detections_to_trackers(detections: np.ndarray, trackers: np.ndarray,
                                     distance_threshold: float = 0.2):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if len(trackers) == 0:
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 3), dtype=int)
    distance_matrix = euclidean_distances(detections, trackers)
    row_matched_indices, col_matched_indices = linear_sum_assignment(distance_matrix)

    unmatched_detections = []
    for d, det in enumerate(detections):
        if d not in row_matched_indices:
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if t not in col_matched_indices:
            unmatched_trackers.append(t)

    # filter out matched with high distance
    matches = []
    for row, col in zip(row_matched_indices, col_matched_indices):
        if distance_matrix[row, col] > distance_threshold:
            unmatched_detections.append(row)
            unmatched_trackers.append(col)
        else:
            matches.append(np.array([row, col]).reshape(1, 2))
    if len(matches) == 0:
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
    def __init__(self, max_age=1, min_hits=3):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
        self.frame_count = 0

    def update(self, dets: np.ndarray) -> np.ndarray:
        """
        @:param: dets - a numpy array of detections in the format [[x,y,phi],[x,y,phi],...]
        :reqires: this method must be called once for each frame even with empty detections.
        :return: numpy array for the states [x,y,omega,x_dot,y_dot,omega_dot] of the tracked objects,
        where the last column is the object ID.

        NOTE: The number of objects returned may differ from the number of detections provided.
        """
        self.frame_count += 1
        # get predicted locations from existing combined_state.
        trks = np.zeros((len(self.trackers), 3))
        to_del = []
        ret = []
        for t, trk in enumerate(trks):
            pos = self.trackers[t].predict()[0]
            trk[:] = [pos[0], pos[1], pos[2]]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets, trks)

        # update matched combined_state with assigned detections
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[0], 0]
                trk.update(dets[d, :][0])

        # create and initialise new combined_state for unmatched detections
        for i in unmatched_dets:
            trk = KalmanTracker(dets[i, :])
            self.trackers.append(trk)
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()[0]
            if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d, [trk.id])).reshape(1, -1))  # +1 as MOT benchmark requires positive
            i -= 1
            # remove dead tracklet
            if trk.time_since_update > self.max_age:
                self.trackers.pop(i)
        if len(ret) > 0:
            return np.concatenate(ret)
        return np.empty((0, 7))
