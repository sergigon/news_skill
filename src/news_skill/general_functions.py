#!/usr/bin/env python
# -*- coding: utf-8 -*-


__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import html2text # To convert html to text (https://pypi.org/project/html2text/)
import rospy

def html2text_conv(html):
    """
    Converts hmtl text into plain text.

    @param html: Html text to convert.

    @return text: Plain text converted.
    """

    # html2text object
    parser = html2text.HTML2Text()
    parser.images_to_alt = True # Discard image data, only keep alt text
    parser.ignore_images = True # Don't include any formatting for images
    parser.ignore_links = True # Ignores links
    parser.ignore_tables = True
    parser.body_width = 1000 # Number of charcaters per line (long number so no '\n' character appears)

    rospy.logdebug('### Content parsed ###')
    text = parser.handle(html)
    # If text ends with '\n', ti is removed
    while(text[-1:] == '\n'):
        text = text[0:-1]
    return text