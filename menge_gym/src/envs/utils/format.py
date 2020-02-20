import numpy as np
import rospy as rp
from typing import Union, List


def format__1(digits, num):
    if digits < len(str(num)):
        raise Exception("digits<len(str(num))")
    return ' ' * (digits - len(str(num))) + str(num)


def format_array(arr: np.ndarray,
                 row_labels: Union[np.ndarray, List] = [], col_labels: Union[np.ndarray, List] = []):
    """
    turns 2d numpy array into str representation with aligned rows and columns and respective labels

    :param arr: np.array with elements to format aligned
    :param row_labels: list of row labels to align left to the first elements of each row
    :param col_labels: list of column labels to align above the first elements of each column

    :returns: formatted str
    """
    arr = np.round(arr, decimals=3)
    num_rows = len(row_labels)
    num_cols = len(col_labels)
    shape = arr.shape
    res_str = ''
    assert num_rows == shape[0], 'Number of row labels (%d) does not match rows in array (%d)' % (num_rows, shape[0])
    assert num_cols == shape[1], 'Number of col labels (%d) does not match cols in array (%d)' % (num_cols, shape[1])
    get_length = np.vectorize(lambda x: len(str(x)), otypes=[int])
    # the maximum number of chars required to display any item in list
    max_chars = np.max(np.concatenate((get_length(arr), get_length(col_labels).reshape(-1, num_cols)), axis=0))
    if not num_rows and not num_cols:
        for row in arr:
            res_str += '[%s]\n' % (' '.join(format__1(max_chars, i) for i in row))
    elif num_rows and num_cols:
        # max char width of row__labels
        rw = np.max(get_length(row_labels))
        res_str += '%s %s\n' % (' ' * (rw + 1), ' '.join(format__1(max_chars, i) for i in col_labels))
        for row_label, row in zip(row_labels, arr):
            res_str += '%s [%s]\n' % (format__1(rw, row_label), ' '.join(format__1(max_chars, i) for i in row))
    else:
        rp.logdebug('arr = %s' % repr(arr))
        rp.logdebug('row_labels = %s' % repr(row_labels))
        rp.logdebug('col_labels = %s' % repr(col_labels))
        raise Exception("This case is not implemented..."
                        "either both row_labels and col_labels must be given or neither.")
    return res_str
