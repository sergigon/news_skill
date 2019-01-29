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

from skill.skill import Skill, ActionlibException, CONDITIONAL
#from time_weather_skill.general_functions import makeCA_info, makeCA_gesture_info # Voice communication CA functions
#from time_weather_skill.sys_operations import SysOperations
#from time_weather_skill.datetime_manager import DatetimeManager
#from time_weather_skill.general_functions import *

import rospkg
import rospy
import roslib
import importlib
import actionlib
#import pdb; pdb.set_trace()

import feedparser
import html2text # To convert html to text (https://pypi.org/project/html2text/)
import datetime
import requests
from io import BytesIO # https://stackoverflow.com/questions/9772691/feedparser-with-timeout
from bs4 import BeautifulSoup # http://villageblacksmith.consulting/extracting-image-from-rss-in-python/

# Messages
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
import newspaper_skill.msg
from common_msgs.msg import KeyValuePair

# Skill variables
# Package name
pkg_name = 'newspaper_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "newspaper_skill"

class NewspaperSkill(Skill):
    """
    Newspaper skill class.
    """

    # Feedback and result of this skill
    _feedback = newspaper_skill.msg.NewspaperFeedback()
    _result = newspaper_skill.msg.NewspaperResult()

    # Node constants
    _ICON_OBTENTION = 0 # Determines the obetention of icons (0: Internet, 1: Local)
    _LANG = 'es' # Language for the conditions
    _COUNTRY = 'es' # Country code (ISO 3166, 2)

    # Feeds
    feeds = {
        'bbc': {
            'portada': 'http://www.bbc.co.uk/mundo/index.xml',
            'lo_ultimo': 'http://www.bbc.co.uk/mundo/ultimas_noticias/index.xml',
            'espana': '',
            'internacional': 'http://www.bbc.co.uk/mundo/temas/internacional/index.xml',
            'opinion': '',
            'deportes': '',
            'tecnologia': 'http://www.bbc.co.uk/mundo/temas/tecnologia/index.xml',
            'ciencia': 'http://www.bbc.co.uk/mundo/temas/ciencia/index.xml',
            'cultura': 'http://www.bbc.co.uk/mundo/temas/cultura/index.xml'
        },
        'europa_press': {
            'portada': 'https://www.europapress.es/rss/rss.aspx',
            'lo_ultimo': '',
            'espana': 'https://www.europapress.es/rss/rss.aspx?ch=00066',
            'internacional': 'https://www.europapress.es/rss/rss.aspx?ch=00069',
            'opinion': '',
            'deportes': 'https://www.europapress.es/rss/rss.aspx?ch=00067',
            'tecnologia': 'https://www.europapress.es/rss/rss.aspx?ch=00564',
            'ciencia': '',
            'cultura': 'https://www.europapress.es/rss/rss.aspx?ch=00126'
        },
        'el_espanol': {
            'portada': '',
            'lo_ultimo': '',
            'espana': '',
            'internacional': '',
            'opinion': '',
            'deportes': '',
            'tecnologia': '',
            'ciencia': '',
            'cultura': ''
        },
        'libertad_digital': {
            'portada': 'http://feeds2.feedburner.com/libertaddigital/portada',
            'lo_ultimo': '',
            'espana': 'http://feeds2.feedburner.com/libertaddigital/nacional',
            'internacional': 'http://feeds2.feedburner.com/libertaddigital/internacional',
            'opinion': 'http://feeds2.feedburner.com/libertaddigital/opinion',
            'deportes': 'http://feeds2.feedburner.com/libertaddigital/deportes',
            'tecnologia': 'http://feeds2.feedburner.com/libertaddigital/internet',
            'ciencia': '',
            'cultura': 'http://feeds.feedburner.com/libertaddigital/cultura'
        },
        'el_pais': {
            'portada': 'http://ep00.epimg.net/rss/elpais/portada.xml',
            'lo_ultimo': 'http://ep00.epimg.net/rss/tags/ultimas_noticias.xml',
            'espana': 'http://ep00.epimg.net/rss/politica/portada.xml',
            'internacional': 'http://ep00.epimg.net/rss/internacional/portada.xml',
            'opinion': 'http://ep00.epimg.net/rss/elpais/opinion.xml',
            'deportes': 'http://ep00.epimg.net/rss/deportes/portada.xml',
            'tecnologia': 'http://ep00.epimg.net/rss/tecnologia/portada.xml',
            'ciencia': 'http://ep00.epimg.net/rss/elpais/ciencia.xml',
            'cultura': 'http://ep00.epimg.net/rss/cultura/portada.xml'
        },
        # No aptos
        '20_minutos': { # Resumenes largos
            'portada': 'https://www.20minutos.es/rss/',
            'lo_ultimo': '',
            'espana': 'https://www.20minutos.es/rss/nacional/',
            'internacional': 'https://www.20minutos.es/rss/internacional/',
            'opinion': 'https://www.20minutos.es/rss/opiniones/',
            'deportes': 'https://www.20minutos.es/rss/deportes/',
            'tecnologia': 'https://www.20minutos.es/rss/tecnologia/',
            'ciencia': 'https://www.20minutos.es/rss/ciencia/',
            'cultura': 'https://www.20minutos.es/rss/cultura/'
        },
        'el_mundo': { # Sin resumenes
            'portada': 'https://e00-elmundo.uecdn.es/elmundo/rss/portada.xml',
            'lo_ultimo': '',
            'espana': 'https://e00-elmundo.uecdn.es/elmundo/rss/espana.xml',
            'internacional': 'https://e00-elmundo.uecdn.es/elmundo/rss/internacional.xml',
            'opinion': '',
            'deportes': 'https://e00-elmundo.uecdn.es/elmundodeporte/rss/portada.xml',
            'tecnologia': '',
            'ciencia': 'https://e00-elmundo.uecdn.es/elmundo/rss/ciencia.xml',
            'cultura': 'https://e00-elmundo.uecdn.es/elmundo/rss/cultura.xml'
        },
    }

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir

        # Paths
        rospack = rospkg.RosPack()
        self._root_path = rospack.get_path(pkg_name) # Package path
        self._data_path = self._root_path + '/data/' # Data path
        self._news_cache = self._data_path + 'news_cache.txt' # Cache file

        # html2text object
        self._parser = html2text.HTML2Text()
        self._parser.images_to_alt = True # Discard image data, only keep alt text
        self._parser.ignore_images = True # Don't include any formatting for images
        self._parser.ignore_links = True # Ignores links
        self._parser.ignore_tables = True
        self._parser.body_width = 1000 # Number of charcaters per line (long number so no '\n' character appears)

        # Rss objects
        self._rss_feed = self.feeds['bbc']['portada'] # Feed url
        self._file_lines = [] # List of cache urls

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        print("create_msg_srv() called")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher("hri_manager/ca_activations", CA, queue_size=10) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher("hri_manager/ca_deactivations", String, queue_size=10) # CA deactivation publisher

        #self.sub_response = rospy.Subscriber(robot + "hri_manager/response", CA, self.response_callback) # CA subscriber

        # servers and clients
        # Si el servidor actionlib no se ha inicializado:
        if not self._as:
            self._as = actionlib.SimpleActionServer(skill_name, newspaper_skill.msg.NewspaperAction, execute_cb=self.execute_cb, auto_start=False)
            # start the action server
            self._as.start()

    def shutdown_msg_srv(self):
        """
        This function has to be implemented in the children.
        """

        # publishers and subscribers
        # FIXME: do not unregister publishers because a bug in ROS

        # servers and clients
            
        print("shutdown_msg_srv() called")

    def html2text_conv(self, html):
        """
        Converts hmtl text into plain text.
        """

        print('### Content parsed ###')
        text = self._parser.handle(html)
        # If text ends with '\n', ti is removed
        while(text[-1:] == '\n'):
            text = text[0:-1]
        return text

    def show_article_info(self, article):
        """
        Show info of the article in the terminal.
        """
        try:
            print('#################### Article: ####################')
            print('\nSummary ' + article['summary'] + '):')
            ###### Title
            print('Title:')
            print(article['title'])
            print('__________________________________________________')
            ###### Id
            print('\nId:')
            print(article['id'])
            print('__________________________________________________')
            ###### Summary
            print('\nSummary (' + article['summary_detail']['type'] + '):')
            # Checks if it is necessary to parse the text
            summary_value = article['summary_detail']['value']
            '''
            if(article['summary_detail']['type'] == 'text/html'):
                summary_value = self.html2text_conv(summary_value)
            '''
            print(summary_value)
            print('__________________________________________________')
            ##### Content
            if 'content' in article:
                for i in range(0, len(article['content'])):
                    print('\nContent[' + str(i) + '](' + article['content'][i]['type'] + '):')
                    content_value = article['content'][i]['value']
                    if(article['content'][i]['type'] == 'text/html'):
                        content_value = self.html2text_conv(content_value)
                    print(content_value)
                print('__________________________________________________')
            ###### Links
            print('\nLinks:')
            print(article['links'])
            print('__________________________________________________')
            ###### Image in summary
            print('\nImage in summary:')
            soup = BeautifulSoup(article['summary'], features="html.parser")
            try:
                image_url = soup.find('img')['src']
            except TypeError as e:
                image_url = False
                rospy.logwarn("No image: %s" % e)
            if image_url:
                print('Image: ' + str(image_url)) # Hey, we have an image!
            else:
                print('No image')
            
            print('__________________________________________________')
            ###### Image
            print('\nImage:')
            if 'media_thumbnail' in article:
                for media in article["media_thumbnail"]:
                    print media
            else:
                print('No image')
            print('##################################################')
        except KeyError as e:
            print('KeyError: ' + str(e))
            return -1

        return 0

    def new_article_finder(self, rss_info):
        '''
        Gets new article. Search in the cache file and if it is not there, it returns the info of the article.
        '''

        # Loop for the entries
        found = False
        for article in rss_info['entries']:
            found = False
            # Checks if id is in the list
            for id_n in self._file_lines:
                if(article['id'] == id_n):
                    found = True
                    break
            # If id is not in the list, actual article is selected
            if(not found):
                return article
                break
        # All articles have been found
        if(found):
            return -1

    def cache_refresh(self):
        """
        Refresh the cache.

        If cache does not exist, it is created.
        If date is not updated, it erases the content and updates the date.
        """

        print('Refreshing cache')

        # Actual date
        date = datetime.datetime.now().date()

        # Reads first line of cache
        first_line = '' # First line of the file
        try:
            file = open(self._news_cache, "r") # Read file
            first_line = file.readline().replace("\n", "") # Gets first line
            file.close() # Close file
        except IOError as e: # File does not exists
            print('ERROR: ' + str(e))

        # If date is not updated or not exists, it erases the file and updates the date
        
        if(first_line[:5] != '_____' or first_line[5:] != str(date)):
            print('Updating cache date')
            file = open(self._news_cache, "w") # Write file
            file.write('_____' + str(date)) # _____{date}
            file.close() # Close file

    def cache_update(self, url):
        '''
        Updates the cache with the new url.
        '''

        file = open(self._news_cache, "w") # Write file
        self._file_lines.append(str(url))
        text = ''
        for line in self._file_lines:
            text += str(line) + '\n'
        file.write(text) # id
        file.close() # Close file

    def rss_reader(self):
        """
        Rss management function.
        """

        # Gets the rss feed info
        try:
            # Do request using requests library and timeout
            print('Making request')
            resp = requests.get(self._rss_feed, timeout=5.05)
        except requests.ReadTimeout as e:
            rospy.logerr("Timeout when reading RSS %s: %s" % self._rss_feed, e)
            return -1
        except requests.exceptions.ConnectionError as e:
            rospy.logerr("Connection error; %s" % e)
            return -1
        except requests.exceptions.MissingSchema as e: # URL not valid
            rospy.logerr("MissingSchema; %s" % e)
            return -1
        print('Request done\n')

        # Puts it to memory stream object universal feedparser
        content = BytesIO(resp.content)
        # Parse content
        d = feedparser.parse(content)

        # Refreshes the cache
        self.cache_refresh()
        
        # Gets cache ids
        file = open(self._news_cache, "r") # Read file
        self._file_lines = file.readlines() # Get lines
        file.close() # Close file
        for i in range(len(self._file_lines)): # Erases the line breaks
            self._file_lines[i] = self._file_lines[i].replace("\n", "")

        # Feed info
        print('############### Feed: ###############')
        print(d['feed']['title'])
        if(d['bozo']==0):
            print 'Xml well-formed'
        else:
            print 'Xml NOT well-formed'

        print('Number of articles: ' + str(len(d['entries'])))
        print('Image: ' + str(d['feed']['image']))

        for key in d:
            if key == 'entries':
                print '>>>>>' + str(key)
                print d[key][0]
                continue
            print '>>>>>' + str(key)
            print d[key]
        for key in d['entries'][0]:
            print '>>>>>' + str(key)
            print d['entries'][0][key]

        ## Image
        # d['entries'][0]["media_thumbnail"][0]["url"]
        # o
        # d['entries'][0]['links'][1]['type'] == image/jpg
        # d['entries'][0]['links'][1]['href']


        print('#####################################')

        # Searchs new article
        article = self.new_article_finder(d)
        if(article == -1): # No more news to show
            print('No more news to show')
            return 0

        # Shows the info in the article
        self.show_article_info(article)
        # Updates the cache with the new article
        self.cache_update(article['id'])
        
        return 0


    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received

        @param goal: newspaper_skill goal.
        """

        # default values (In progress)
        self._result.result = -1 # Error
        self._feedback.feedback = 0

        ############### Si la skill esta activa: ###################
        if self._status == self.RUNNING:
            print ("RUNNING...")

            try:
                ############# State Preempted checking #############
                # Si el goal esta en estado Preempted (es decir,   #
                # hay un goal en cola o se cancela el goal         #
                # actual), activo la excepcion                     #
                ####################################################
                if self._as.is_preempt_requested():
                    print("Preempt requested")
                    raise ActionlibException
                #==================================================#
                
                ################ Processes the goal ################
                print(goal.command)
                self._result.result = self.rss_reader()
                #==================================================#
            
            ######### Si se ha hecho un preempted o cancel: ########
            except ActionlibException:
                rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
                # FAIL
                self._result.result = 1 # Preempted
            #======================================================#
            
        #==========================================================#
        ############# Si la skill no esta activa: ##################
        else:
            print("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            # ERROR
            self._result.result = -1 # Fail
        #==========================================================#
        
        # Envio el feedback al final del loop
        self._as.publish_feedback(self._feedback)
        
        #### Envio del resultado y actualizacion del status del goal ###
        # Send result
        if self._result.result == 0:
            rospy.logdebug("setting goal to succeeded")
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._as.set_preempted(self._result)
        print("###################")
        print("### Result sent ###")
        print("###################")
        #==============================================================#


        # Inicializacion variables



if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name, log_level=rospy.DEBUG)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = NewspaperSkill()
        rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
