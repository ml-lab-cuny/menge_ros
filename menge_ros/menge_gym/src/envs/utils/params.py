import xml.etree.ElementTree as ElT
from os.path import isfile
import numpy as np
import rospy as rp
from typing import Union


def parseXML(xml_file: str) -> ElT:
    """
    parse an xml file

    :param xml_file:    str, path to xml file
    :return: ElementTree root
    """
    assert isfile(xml_file), "provided argument xml_file does not point to a valid file"
    tree = ElT.parse(xml_file)
    root = tree.getroot()

    return root


def constraints_fulfilled(constraints: dict = None, attributes: dict = None) -> bool:
    if constraints and attributes:
        return set(constraints.items()).issubset(attributes.items())
    elif not constraints:
        return True
    else:
        return False


def match_in_tree(tree, tag: str = None, attrib_name: str = None,
                  constraints: dict = None, return_all: bool = False) -> (bool, Union[str, dict]):
    """
    returns first occurrence of a value in ElementTree that where tag and/or name of attribute match

    :param tree:            xml.etree.ElementTree to look in for value
    :param tag:             str, tag to match in ElementTree
    :param attrib_name:     str, attribute name to match in ElementTree
    :param constraints:     dict {attrib_name: value}, constraints, that need to be matched as well
    :param return_all:      bool, if True returns all matches, else returns first

    :return:
        success:    bool if value has been found
        attributes:
            if return_all == False --> either dict of attributes (if only tag specified)
                                        or single value where tag and attrib_name match
                                        None, if nothing found
            else                   --> list of results (elements like when return_all == False)
    """
    results = [] if return_all else None

    if not tag and not attrib_name:
        rp.logdebug('Neither tag nor attrib_name defined. Nothing to match')
        return False, None

    elif tag:
        # match tag
        if tree.tag == tag:
            if attrib_name:
                for attrib in tree.attrib:
                    # match attribute and constraints
                    if attrib == attrib_name and constraints_fulfilled(constraints, tree.attrib):
                        if return_all:
                            # add attribute value to results
                            results.append(tree.attrib[attrib_name])
                        else:
                            # return attribute value
                            return True, tree.attrib[attrib_name]
            # matching tag, no attribute
            else:
                if return_all:
                    # add dict of attributes to results
                    results.append(tree.attrib)
                else:
                    # return dict of attributes
                    return True, tree.attrib
    # no tag, only attribute
    elif attrib_name:
        for attrib in tree.attrib:
            # match attribute and constraints
            if attrib == attrib_name and constraints_fulfilled(constraints, tree.attrib):
                if return_all:
                    # add attribute value to results
                    results.append(tree.attrib[attrib_name])
                else:
                    # return attribute value
                    return True, tree.attrib[attrib_name]
    # if child elements
    if list(tree):
        # check each child
        for child in tree:
            success, attributes = match_in_tree(child, tag, attrib_name, constraints, return_all)
            if success:
                if return_all:
                    results.extend(attributes)
                else:
                    return True, attributes
        if not return_all:
            return False, None
    if results:
        return True, results
    else:
        return False, results


def match_in_xml(xml_file, tag: str = None, attrib_name: str = None,
                 constraints: dict = None, return_all: bool = False) -> Union[str, dict]:
    """

    :param xml_file:        str, path to xml file
    :param tag:             str, tag to match in xml file
    :param attrib_name:     str, attribute name to match in xml file
    :param constraints:     dict {attrib_name: value}, constraints, that need to be matched as well
    :param return_all:      bool, if True returns all matches, else returns first

    :return:
        attributes:
            if return_all == False --> either dict of attributes (if only tag specified)
                                        or single value where tag and attrib_name match
                                        None, if nothing found
            else                   --> list of results (elements like when return_all == False)
    """

    root = parseXML(xml_file)
    _, attributes = match_in_tree(root, tag, attrib_name, constraints, return_all)

    return attributes


def goal2array(goal: dict) -> np.ndarray:
    """
    transforms a Menge goal passed as a dict into a numpy array (turning it into circular goal regardless of type before)

    :param goal:    dict, specifying goal
    :return: np.array [x, y, r], specifying goal as center x,y and radius r
    """

    if goal['type'].lower() == 'point':
        center_x = float(goal['x'])
        center_y = float(goal['y'])
        radius = 0.0
    elif goal['type'].lower() == 'circle':
        center_x = float(goal['x'])
        center_y = float(goal['y'])
        radius = float(goal['radius'])
    elif goal['type'].lower() == 'obb':
        width = float(goal['width'])
        height = float(goal['height'])
        angle_rad = float(goal['angle']) * np.pi / 180
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        center_x = float(goal['x']) + cos_a * width / 2 - sin_a * height / 2
        center_y = float(goal['y']) + cos_a * height / 2 + sin_a * width / 2
        radius = min(width, height)
    elif goal['type'].lower() == 'aabb':
        min_p = np.array([float(goal['min_x']), float(goal['min_y'])])
        max_p = np.array([float(goal['max_x']), float(goal['max_y'])])
        dims = max_p - min_p
        radius = dims.min()
        center_x, center_y = min_p + dims / 2
    else:
        raise ValueError("invalid GoalType")

    return np.array([center_x, center_y, radius])
