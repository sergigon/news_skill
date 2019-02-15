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
import signal
import time
from io import BytesIO # https://stackoverflow.com/questions/9772691/feedparser-with-timeout
from bs4 import BeautifulSoup # http://villageblacksmith.consulting/extracting-image-from-rss-in-python/

# Messages
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
from common_msgs.msg import KeyValuePair as kvpa
import newspaper_skill.msg

import exceptions_lib

# Skill variables
# Package name
pkg_name = 'newspaper_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "newspaper_skill"

'''# Exceptions
class TimeOut(Exception):
    __module__ = Exception.__module__
    pass
'''

class NewspaperSkill(Skill):
    """
    Newspaper skill class.
    """

    # Feedback and result of this skill
    _feedback = newspaper_skill.msg.NewspaperFeedback()
    _result = newspaper_skill.msg.NewspaperResult()

    # Node constants

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
        'tu_tiempo': {
            'portada': 'http://xml.tutiempo.net/xml/3768.xml'
        }
    }
    feeds_no = {
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
        'el_espanol': { # Esta vacio
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
        'agencia_efe': { # Sin resumen
            'portada': 'https://www.efe.com/efe/espana/1/rss'
        }
    }

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir

        # Local paths
        rospack = rospkg.RosPack()
        self._root_path = rospack.get_path(pkg_name) # Package path
        self._data_path = self._root_path + '/data/' # Data path
        self._news_cache = self._data_path + 'news_cache.txt' # Cache file
        # Tablet paths
        self._default_path = 'default.png' # Default image

        # html2text object
        self._parser = html2text.HTML2Text()
        self._parser.images_to_alt = True # Discard image data, only keep alt text
        self._parser.ignore_images = True # Don't include any formatting for images
        self._parser.ignore_links = True # Ignores links
        self._parser.ignore_tables = True
        self._parser.body_width = 1000 # Number of charcaters per line (long number so no '\n' character appears)

        # Rss objects
        self._feed_name = 'tu_tiempo'
        self._category_name = 'portada'
        #self._rss_feed = self.feeds[self._feed_name][self._category_name] # Feed url
        self._file_lines = [] # List of cache urls

        # Register the signal function handler
        signal.signal(signal.SIGALRM, self.handler)

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        print("create_msg_srv() called")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher("hri_manager/ca_activations", CA, queue_size=1) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher("hri_manager/ca_deactivations", String, queue_size=1) # CA deactivation publisher

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

    def handler(self, signum, frame):
        """
        Register an handler for the timeout.
        """

        print "Forever is over!"
        raise exceptions_lib.TimeOut("end of time")

    def makeCA_etts_info(self, etts_text):
        now = rospy.get_rostime().nsecs

        msg = CA()
        msg.type = "robot_giving_info"
        msg.ca_name = str(now)
        msg.duration = 0
        msg.priority = 1
        msg.emitter = "newspaper_ca"
        
        kvp = kvpa()
        kvp.key = "etts_text"
        kvp.value = etts_text
        msg.values.append(kvp)
        print "Sending CA_info"
        return msg

    def makeCA_tablet_info(self, image_url, image_type):
        now = rospy.get_rostime().nsecs

        msg = CA()
        msg.type = "robot_giving_info"
        msg.ca_name = str(now)
        msg.duration = 0
        msg.priority = 1
        msg.emitter = "newspaper_ca"
        
        kvp = kvpa()
        kvp.key = "tablet_type"
        kvp.value = image_type
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "tablet_url"
        kvp.value = image_url
        msg.values.append(kvp)
        print "Sending CA_info"
        return msg


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

    def get_image(self, article):
        if(article == -1):
            return -1, -1
        image_url, image_type = -1, -1

        # Search in summary
        rospy.logdebug('Searching image in summary')
        if 'summary' in article:
            soup = BeautifulSoup(article['summary'], features="html.parser")
            try:
                image_url = soup.find('img')['src']
                rospy.logdebug('Image selected from summary: %s' % image_url)
            except TypeError as e: # Non existing
                image_url = -1
                rospy.logwarn('No image in summary')
        else:
            rospy.logwarn('No summary')

        # Media thumbnail
        found = False
        rospy.logdebug('Searching image in media thumbnail')
        if 'media_thumbnail' in article:
            image_url = article['media_thumbnail'][0]['url']
            rospy.logdebug('Image selected from media_thumbnail: %s' % image_url)
            found = True
            if(len(article['media_thumbnail']) > 1):
                rospy.logwarn('More than 1 media thumbnail: %s' % len(article['media_thumbnail']))
        if(not found):
            rospy.logwarn('No image in media thumbnail')

        # Media content
        found = False
        rospy.logdebug('Searching image in media content')
        if 'media_content' in article:
            print str(article['media_content'])
            image_url = article['media_content'][0]['url']
            rospy.logdebug('Image selected from media_content: %s' % image_url)
            found = True
            if(len(article['media_content']) > 1):
                rospy.logwarn('More than 1 media content: %s' % len(article['media_content']))
        if(not found):
            rospy.logwarn('No image in media content')

        # Links
        found = False
        rospy.logdebug('Searching image in links')
        if 'links' in article:
            for link in article['links']:
                if(link['type'].find('image') >= 0): # It is a image
                    image_url = link['href']
                    rospy.logdebug('Image selected from links: %s' % image_url)
                    found = True
                    ######## Europa Press conversion ########
                    if(self._feed_name == 'europa_press'):
                        rospy.logdebug('Europa Press conversion')
                        rospy.logdebug('Original link: ' + image_url)
                        # Searchs the position of the extension
                        pos_l = len(image_url)-5 # Indicates how many character to search from the end of string
                        pos_r = pos_l + image_url[pos_l:].find('.') # Position of the extension ('.jpg')
                        # Searchs if exists 'vX' at the end
                        pos_l = pos_r-4
                        pos_aux = image_url[pos_l:pos_r].find('_v')
                        if (pos_aux !=-1):
                            pos_r = pos_l + pos_aux
                        # Searchs if exists '_XXX' at the end
                        pos_l = pos_r-5
                        pos_aux = image_url[pos_l:pos_r].find('_')
                        if (pos_aux !=-1):
                            pos_l = pos_l + pos_aux
                            # Modify the size of the image
                            s1 = list(image_url)
                            for i in range(pos_l+1, pos_r):
                                s1.pop(pos_l+1)
                            s1.insert(pos_l+1, '800')
                            image_url = ''.join(s1)
                        rospy.logdebug('Modified link: ' + image_url)
                    ##########################################
                    break
        if(not found):
            rospy.logwarn('No image in links')

        if image_url == -1:
            return self._default_path, 'image'
        else:
            return image_url, 'web'

    def show_article_info(self, article):
        """
        Show info of the article in the terminal.
        """
        if(article == -1):
            print('No more articles to show in the category %s' % self._category_name)
            return -1

        try:
            print('#################### Article: ####################')

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
            if(article['summary_detail']['type'] == 'text/html'):
                summary_value = self.html2text_conv(summary_value)
            print(summary_value)
            print('__________________________________________________')

            print('##################################################')
        except KeyError as e:
            print('KeyError: ' + str(e))
            return -1

        return 0

    def show_article_voice(self, article):
        print('Say article with voice')
        if(article == -1):
            msg = self.makeCA_etts_info('No hay mas noticias que mostrar del dia de hoy')
            self.ca_pub.publish(msg)
            return -1
        try:
            ###### Title
            print('Title voice:')
            msg = self.makeCA_etts_info(article['title'])
            self.ca_pub.publish(msg)
            print('__________________________________________________')
            ###### Summary
            print('\nSummary (' + article['summary_detail']['type'] + ') voice:')
            # Checks if it is necessary to parse the text
            summary_value = article['summary_detail']['value']
            if(article['summary_detail']['type'] == 'text/html'):
                summary_value = self.html2text_conv(summary_value)
            msg = self.makeCA_etts_info(summary_value)
            self.ca_pub.publish(msg)
            print('__________________________________________________')
        except KeyError as e:
            print('KeyError: ' + str(e))
            return -1

        return 0


    def new_article_finder(self, rss_info):
        '''
        Gets new article. Search in the cache file and if it is not there, it returns the info of the article.
        '''

        if(len(rss_info['entries']) == 0):
            return -2 # Rss not valid

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

    def cache_get_id(self):
        """
        Gets id from the cache and export them to the variable self._file_lines.
        """

        file = open(self._news_cache, "r") # Read file
        self._file_lines = file.readlines() # Get lines
        file.close() # Close file
        for i in range(len(self._file_lines)): # Erases the line breaks
            self._file_lines[i] = self._file_lines[i].replace("\n", "")

    def rss_reader(self):
        """
        Rss management function.
        """

        # Search for the next source feed
        for feed_name in self.feeds:
            if self._category_name in self.feeds[feed_name]:
                ################### Feed found ###################
                print('######## Getting new feed: %s ########' % feed_name)
                self._feed_name = feed_name
                feed_url = self.feeds[self._feed_name][self._category_name]

                ############ Request ############
                # Gets the rss feed info
                out = False
                n_attempts = 0
                n_attempts_max = 3
                request_ok = False
                while(not out):
                    rospy.logdebug('Number of attempts: %s' % n_attempts)
                    try:
                        # Do request using requests library and timeout
                        print('Making request')
                        resp = requests.get(feed_url, timeout=5.05)
                    except requests.ReadTimeout as e:
                        rospy.logerr("Timeout when reading RSS: %s" % e)
                        n_attempts += 1
                    except requests.exceptions.ConnectionError as e:
                        rospy.logerr("Connection error; %s" % e)
                        n_attempts += 1
                    except requests.exceptions.MissingSchema as e: # URL not valid
                        rospy.logerr("MissingSchema; %s" % e)
                        request_ok = False
                        out = True

                    else: # No errors
                        print('Request done\n')
                        request_ok = True
                        out = True

                    signal.alarm(0) # Disable the alarm
                    if(n_attempts>=n_attempts_max):
                        rospy.logwarn('attempts (%s) >= attempts max (%s)' % (n_attempts, n_attempts_max))
                        request_ok = False
                        out = True

                if(not request_ok):
                    continue
                #################################

                ############ Parsing ############
                # Puts it to memory stream object universal feedparser
                content = BytesIO(resp.content)
                # Parse content
                parsed_content = feedparser.parse(content)
                #################################
                #print parsed_content##############################

                ######### Cache checking ########
                # Refreshes the cache
                self.cache_refresh()

                # Gets cache ids
                self.cache_get_id()

                # Searchs new article
                article = self.new_article_finder(parsed_content)
                #################################
                ######### Article found #########
                if(article != -1 and article != -2):
                    # Feed info
                    print('############### Feed: ###############')
                    try:
                        print(parsed_content['feed']['title'])
                        if(parsed_content['bozo']==0):
                            print 'Xml well-formed'
                        else:
                            print 'Xml NOT well-formed'

                        print('Number of articles: ' + str(len(parsed_content['entries'])))
                        print('Image: ' + str(parsed_content['feed']['image']['href']))
                    except KeyError as e:
                        print('KeyError: ' + str(e))

                    print('#####################################')


                    # Shows the info in the article
                    self.show_article_info(article)

                    # Shows image on the tablet
                    print('Searching article image')
                    image_url, image_type = self.get_image(article)
                    print image_url, image_type
                    print('Sending image to tablet')
                    if(image_url != -1):
                        msg = self.makeCA_tablet_info(image_url, image_type)
                        self.ca_pub.publish(msg)
                    else:
                        print('No image sent to the tablet')

                    # Shows info with voice
                    self.show_article_voice(article)

                    # Updates the cache with the new article
                    if(article != -1):
                        self.cache_update(article['id'])

                    #print str(article)
                    return 0
                #################################
                ######### Feed not valid ########
                if(article==-2):
                    rospy.logwarn('Feed not valid')
                #################################
                ####### Article NOT found #######
                else:
                    rospy.logwarn('No more articles in the feed')
                #################################

        self.show_article_voice(-1)
        self.show_article_info(-1)
        
        return 0

    def goal_handler(self, goal):
        """
        Goal handler.

        Checks if the goal is appropriate. If true, it fills the variable self._category_name.
        """

        if(goal == 'portada'
            or goal == 'lo_ultimo'
            or goal == 'espana'
            or goal == 'internacional'
            or goal == 'opinion'
            or goal == 'deportes'
            or goal == 'tecnologia'
            or goal == 'ciencia'
            or goal == 'cultura'):
            print('goal accepted')
            self._category_name = goal
            return True

        else:
            print('goal NOT accepted')
            self._category_name = ''
            return False

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received.

        @param goal: newspaper_skill goal.
        """

        # default values (In progress)
        self._result.result = -1 # Error
        self._feedback.status = 0

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
                if(self.goal_handler(goal.command)):
                    t_max = 20
                    n_games = 5
                    i_games = 0
                    t1 = time.time()
                    t2 = time.time()
                    while(i_games < n_games and t2-t1 < t_max):
                        self._result.result = self.rss_reader()
                        rospy.sleep(5)
                        i_games += 1
                        t2 = time.time()
                        print ('>>>>> Time!!!! %s'%(t2-t1))
                else:
                    self._result.result = -1 # Error
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