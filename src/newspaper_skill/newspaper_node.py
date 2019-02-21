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
import requests
import signal
import time
from threading import Thread
from io import BytesIO # https://stackoverflow.com/questions/9772691/feedparser-with-timeout
from bs4 import BeautifulSoup # http://villageblacksmith.consulting/extracting-image-from-rss-in-python/

# Messages
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
import newspaper_skill.msg

# Local libraries
import exceptions_lib
from ca_functions import makeCA_tablet_info, makeCA_etts_info
from general_functions import html2text_conv
from cache_manager import CacheManager

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
class PauseException(Exception):
    pass

class ErrorException(Exception):
    pass

class ExceptThread(Thread):
    def run(self):
        ########### Pause ###########
        while(self._pause and not self._as.is_preempt_requested()): # Pause and cancel not requested
            rospy.logwarn("Pause requested")
            rospy.sleep(1)

        ############# State Preempted checking #############
        # Si el goal esta en estado Preempted (es decir,   #
        # hay un goal en cola o se cancela el goal         #
        # actual), activo la excepcion                     #
        ####################################################
        if self._as.is_preempt_requested():
            rospy.logwarn("Preempt requested")
            raise ActionlibException
        #==================================================#

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
            'cover_page': 'http://www.bbc.co.uk/mundo/index.xml',
            'last_news': 'http://www.bbc.co.uk/mundo/ultimas_noticias/index.xml',
            'spain': '',
            'international': 'http://www.bbc.co.uk/mundo/temas/internacional/index.xml',
            'opinion': '',
            'sports': '',
            'technology': 'http://www.bbc.co.uk/mundo/temas/tecnologia/index.xml',
            'science': 'http://www.bbc.co.uk/mundo/temas/ciencia/index.xml',
            'culture': 'http://www.bbc.co.uk/mundo/temas/cultura/index.xml'
        },
        'europa_press': {
            'cover_page': 'https://www.europapress.es/rss/rss.aspx',
            'last_news': '',
            'spain': 'https://www.europapress.es/rss/rss.aspx?ch=00066',
            'international': 'https://www.europapress.es/rss/rss.aspx?ch=00069',
            'opinion': '',
            'sports': 'https://www.europapress.es/rss/rss.aspx?ch=00067',
            'technology': 'https://www.europapress.es/rss/rss.aspx?ch=00564',
            'science': '',
            'culture': 'https://www.europapress.es/rss/rss.aspx?ch=00126'
        },
        'libertad_digital': {
            'cover_page': 'http://feeds2.feedburner.com/libertaddigital/portada',
            'last_news': '',
            'spain': 'http://feeds2.feedburner.com/libertaddigital/nacional',
            'international': 'http://feeds2.feedburner.com/libertaddigital/internacional',
            'opinion': 'http://feeds2.feedburner.com/libertaddigital/opinion',
            'sports': 'http://feeds2.feedburner.com/libertaddigital/deportes',
            'technology': 'http://feeds2.feedburner.com/libertaddigital/internet',
            'science': '',
            'culture': 'http://feeds.feedburner.com/libertaddigital/cultura'
        },
        'el_pais': {
            'cover_page': 'http://ep00.epimg.net/rss/elpais/portada.xml',
            'last_news': 'http://ep00.epimg.net/rss/tags/ultimas_noticias.xml',
            'spain': 'http://ep00.epimg.net/rss/politica/portada.xml',
            'international': 'http://ep00.epimg.net/rss/internacional/portada.xml',
            'opinion': 'http://ep00.epimg.net/rss/elpais/opinion.xml',
            'sports': 'http://ep00.epimg.net/rss/deportes/portada.xml',
            'technology': 'http://ep00.epimg.net/rss/tecnologia/portada.xml',
            'science': 'http://ep00.epimg.net/rss/elpais/ciencia.xml',
            'culture': 'http://ep00.epimg.net/rss/cultura/portada.xml'
        },
        'tu_tiempo': {
            'cover_page': 'http://xml.tutiempo.net/xml/3768.xml'
        }
    }
    feeds_no = {
        # No aptos
        '20_minutos': { # Resumenes largos
            'cover_page': 'https://www.20minutos.es/rss/',
            'last_news': '',
            'spain': 'https://www.20minutos.es/rss/nacional/',
            'international': 'https://www.20minutos.es/rss/internacional/',
            'opinion': 'https://www.20minutos.es/rss/opiniones/',
            'sports': 'https://www.20minutos.es/rss/deportes/',
            'technology': 'https://www.20minutos.es/rss/tecnologia/',
            'science': 'https://www.20minutos.es/rss/ciencia/',
            'culture': 'https://www.20minutos.es/rss/cultura/'
        },
        'el_mundo': { # Sin resumenes
            'cover_page': 'https://e00-elmundo.uecdn.es/elmundo/rss/portada.xml',
            'last_news': '',
            'spain': 'https://e00-elmundo.uecdn.es/elmundo/rss/espana.xml',
            'international': 'https://e00-elmundo.uecdn.es/elmundo/rss/internacional.xml',
            'opinion': '',
            'sports': 'https://e00-elmundo.uecdn.es/elmundodeporte/rss/portada.xml',
            'technology': '',
            'science': 'https://e00-elmundo.uecdn.es/elmundo/rss/ciencia.xml',
            'culture': 'https://e00-elmundo.uecdn.es/elmundo/rss/cultura.xml'
        },
        'el_espanol': { # Esta vacio
            'cover_page': '',
            'last_news': '',
            'spain': '',
            'international': '',
            'opinion': '',
            'sports': '',
            'technology': '',
            'science': '',
            'culture': ''
        },
        'agencia_efe': { # Sin resumen
            'cover_page': 'https://www.efe.com/efe/espana/1/rss'
        }
    }

    def __init__(self):
        """
        Init method.
        """

        # class variables
        self._as = None # SimpleActionServer variable
        self._goal = "" # Goal a recibir
        self._goal_exec = False # Indicates if goal is being handled
        self._pause = False # Indicates if pause
        self._step = 'init' # Init, rss_reader, show_info
        self._out = False # Varable to exit the while

        # Goal varaibles
        self._max_time = 0
        self._number_plays = 0
        self._t0 = 0
        self._t1 = 0
        self._time_run = 0
        self._i_plays = 0

        # Local paths
        rospack = rospkg.RosPack()
        self._root_path = rospack.get_path(pkg_name) # Package path
        self._data_path = self._root_path + '/data/' # Data path
        self._news_cache = self._data_path + 'news_cache.txt' # Cache file
        # Tablet paths
        self._default_path = '/default.png' # Default image

        # Rss objects
        self._feed_name = 'tu_tiempo'
        self._category_name = 'cover_page'
        #self._rss_feed = self.feeds[self._feed_name][self._category_name] # Feed url

        # Cache object
        self._cache_manager = CacheManager(self._data_path)

        # Register the signal function handler
        signal.signal(signal.SIGALRM, self.handler)

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)


    def create_msg_srv(self):
        """
        This function has to be implemented in the children.

        @raise rospy.ROSException: if the service is inactive.
        """
        rospy.logdebug("create_msg_srv() called")

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

        rospy.logdebug("shutdown_msg_srv() called")

        # Cancel goal
        self._as.preempt_request = True

        # servers and clients
        

    def handler(self, signum, frame):
        """
        Register an handler for the timeout.
        """

        print "Forever is over!"
        raise exceptions_lib.TimeOut("end of time")

    def get_image(self, article):
        """
        Searchs for the best image in the article.

        @param article: Article to search in.
        
        @return image_url: Url of image found. If not found, it returns default image.
        @return image_type: Type of image found ('web' or 'image').
        """

        image_url, image_type = -1, 'web'

        # Search in summary
        rospy.logdebug('Searching image in summary')
        if 'summary' in article:
            soup = BeautifulSoup(article['summary'], features="html.parser")
            try:
                image_url = soup.find('img')['src']
                rospy.logdebug('>> Image selected from summary: %s' % image_url)
            except TypeError as e: # Non existing
                image_url = -1
                rospy.logdebug('>> No image in summary')
        else:
            rospy.logwarn('>> No summary')

        # Media thumbnail
        found = False
        rospy.logdebug('Searching image in media thumbnail')
        if 'media_thumbnail' in article:
            image_url = article['media_thumbnail'][0]['url']
            rospy.logdebug('>> Image selected from media_thumbnail: %s' % image_url)
            found = True
            if(len(article['media_thumbnail']) > 1):
                rospy.logdebug('>> More than 1 media thumbnail: %s' % len(article['media_thumbnail']))
        if(not found):
            rospy.logdebug('>> No image in media thumbnail')

        # Media content
        found = False
        rospy.logdebug('Searching image in media content')
        if 'media_content' in article:
            image_url = article['media_content'][0]['url']
            rospy.logdebug('>> Image selected from media_content: %s' % image_url)
            found = True
            if(len(article['media_content']) > 1):
                rospy.logdebug('>> More than 1 media content: %s' % len(article['media_content']))
        if(not found):
            rospy.logdebug('>> No image in media content')

        # Links
        found = False
        rospy.logdebug('Searching image in links')
        if 'links' in article:
            for link in article['links']:
                if(link['type'].find('image') >= 0): # It is a image
                    image_url = link['href']
                    rospy.logdebug('>> Image selected from links: %s' % image_url)
                    found = True
                    ######## Europa Press conversion ########
                    if(self._feed_name == 'europa_press'):
                        rospy.logdebug('>> Europa Press conversion')
                        rospy.logdebug('>> -- Original link: ' + image_url)
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
                        rospy.logdebug('>> -- Modified link: ' + image_url)
                    ##########################################
                    break
        if(not found):
            rospy.logdebug('>> No image in links')

        if image_url == -1:
            return self._default_path, 'image'
        else:
            return image_url, 'web'

    def show_feed_info(self, parsed_content):
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
            rospy.logerr('KeyError: ' + str(e))

        print('#####################################')


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
                summary_value = html2text_conv(summary_value)
            print(summary_value)

            print('##################################################')
        except KeyError as e:
            rospy.logerr('KeyError: ' + str(e))
            return -1

        return 0

    def show_article_voice(self, article):
        rospy.loginfo('Showing article with voice')
        if(article == -1):
            text = 'No hay mas noticias que mostrar del dia de hoy'
            rospy.loginfo('>> %s' % text)
        try:
            ###### Title
            title_text = article['title'].encode('utf-8')
            rospy.loginfo('>> Title: ' + title_text)
            ###### Summary
            # Checks if it is necessary to parse the text
            summary_value = article['summary_detail']['value']
            if(article['summary_detail']['type'] == 'text/html'):
                summary_value = html2text_conv(summary_value)
            rospy.loginfo('>> Summary (%s): ' % article['summary_detail']['type'])
            summary_text = summary_value.encode('utf-8')
            rospy.loginfo(summary_text)
            text = title_text + ' \\\\pause=1000 ' + summary_text

        except KeyError as e:
            rospy.logerr('KeyError: ' + str(e))
            return -1
        
        msg = makeCA_etts_info(text)
        self.ca_pub.publish(msg)

        return msg.ca_name

    def show_article_tablet(self, image_url, image_type):
        rospy.loginfo('Sending image to tablet')
        msg = makeCA_tablet_info(image_url, image_type)
        self.ca_pub.publish(msg)
        return msg.ca_name

    def new_article_finder(self, rss_info):
        '''
        Gets new article. Search in the cache file and if it is not there, it returns the info of the article.
        '''

        if(len(rss_info['entries']) == 0):
            return -2 # Rss not valid

        # Gets cache ids
        id_list = self._cache_manager.cache_get_id()

        # Loop for the entries
        found = False
        for article in rss_info['entries']:
            found = False
            # Checks if id is in the list
            for id_n in id_list:
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

    def rss_reader(self):
        """
        Rss management function.

        @return parsed_content:
        @return article:
        """

        ######### Cache refreshing ########
        # Refreshes the cache
        self._cache_manager.cache_refresh()
        ###################################

        # Search for the next source feed
        feed_list = [self._feed_name]
        for feed_name in self.feeds:
            if feed_name == self._feed_name:
                continue
            feed_list.append(feed_name)
        print feed_list
        for feed_name in feed_list:
            if self._category_name in self.feeds[feed_name]:
                ################### Feed found ###################
                rospy.loginfo('######## Getting new feed: %s ########' % feed_name)
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
                        rospy.loginfo('Making request')
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
                        rospy.logdebug('Request done')
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

                ########## Article search #########
                # Searchs new article
                article = self.new_article_finder(parsed_content)
                ###################################

                ######### Article found #########
                if(article != -1 and article != -2):
                    return parsed_content, article
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
        
        return -1, -1

    def show_info_handler(self, parsed_content, article, image_url, image_type):
        """
        Show info handler.

        @param art
        """
        # Feed info
        self.show_feed_info(parsed_content)

        # Shows the info in the article
        self.show_article_info(article)

        # Shows image on the tablet
        tablet_name_msg = self.show_article_tablet(image_url, image_type)

        # Shows info with voice
        etts_name_msg = self.show_article_voice(article)

        return tablet_name_msg, etts_name_msg

    def goal_handler(self, goal):
        """
        Goal handler.

        Checks if the goal is appropriate. If true, it fills the variable self._category_name.
        """

        # Fill variables
        skill_command_vec = goal.skill_command.split('/')
        if(len(skill_command_vec)<=1):
            return False
        self._feed_name = skill_command_vec[0]
        self._category_name = skill_command_vec[1]
        self._max_time = goal.max_time
        self._number_plays = goal.number_plays

        # Check feed name
        if not self._feed_name in self.feeds:
            rospy.logerr('feed name NOT accepted')
            return False

        # Check category name
        # Gets the name of one feed to get category access
        for feed in self.feeds:
            feed_name = feed
            break
        if not self._category_name in self.feeds[feed]:
            rospy.logerr('category name NOT accepted')
            return False

        # Check max_time and number_plays
        if(self._max_time>0 and self._number_plays>0):
            self._limit_method = 'both'
        elif(self._max_time>0):
            self._limit_method = 'time'
        elif(self._number_plays>0):
            self._limit_method = 'plays'
        else:
            rospy.logerr('max_time and number_plays NOT accepted. Specify one of them')
            return False

        # Goal accepted
        rospy.loginfo('Goal accepted')
        return True

    def exception_check(self, deactivation = [], t0 = -1, t1 = -1):

        if(self._pause or self._as.is_preempt_requested()):
            # CA deactivation
            if(len(deactivation)>0):
                print 'deactivation'
                for msg_name in deactivation:
                    rospy.logdebug('Deactivating CA: %s' % msg_name)
                    self.ca_deactivation_pub.publish(msg_name)
            # Time update
            if(t0!=-1 and t1!=-1):
                self._time_run += t1 - t0

        ############# State Preempted checking #############
        # Si el goal esta en estado Preempted (es decir,   #
        # hay un goal en cola o se cancela el goal         #
        # actual), activo la excepcion                     #
        ####################################################
        if(self._as.is_preempt_requested()):
            rospy.logwarn("Preempt requested")
            raise ActionlibException
            return
        #==================================================#
        
        ########### Pause ###########
        if(self._pause):
            raise PauseException
            return

    def pause_exec(self):
        if(self._goal_exec): # Goal being handled
            self._pause = True
        else: # Goal NOT being handled
            rospy.logwarn('Goal not being handled')

    def resume_exec(self):
        self._pause = False
        self._feedback.app_status = 'resume_ok'
        self._as.publish_feedback(self._feedback)

    def pause_wait(self):
        rospy.loginfo('Start waiting')
        while(self._pause and not self._as.is_preempt_requested()):
            rospy.logdebug('waiting...')
            rospy.sleep(1)

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received.

        @param goal: newspaper_skill goal.
        """

        self._pause = False
        self._step = 'Process_goal'
        self._limit_method = ''
        self._percentage = 0
        self._time_run, self._i_plays = 0, 0
        self._exec_out = False
        percentage_plays, percentage_time = 0,0

        # Result default values
        self._result.skill_result = self._result.SUCCESS # Success
        # Feedback default values
        self._feedback.app_status = 'start_ok'
        self._feedback.percentage_completed = 0
        self._feedback.engagement = True
        self._as.publish_feedback(self._feedback)

        ############### Si la skill esta activa: ###################
        if self._status == self.RUNNING:
            self._goal_exec = True # Goal execution starts
            print('\n')
            rospy.loginfo("RUNNING...")
            ###################### Exec while ######################
            while(not self._exec_out):
                try:
                    # Wait loop
                    self.pause_wait() # If wait is asked it enters a loop

                    self.exception_check() # Checks if a exception is requested

                    # Process_goal
                    if(self._step=='Process_goal'):
                        ############ Processes the goal ############
                        rospy.loginfo('Goal: %s' % goal)
                        if(not self.goal_handler(goal)): # Goal NOT correct
                            raise ErrorException('Goal NOT correct')

                        self._step = 'Get_rss_info'
                        #==========================================#
                    self.exception_check() # Checks if a exception is requested

                    t0 = time.time()

                    # Get_rss_info
                    if(self._step=='Get_rss_info'):
                        # Get rss info
                        parsed_content, article = self.rss_reader()
                        if(parsed_content==-1):
                            raise ErrorException('No More News')
                            break
                        self._step = 'Search_image'
                    self.exception_check(t0 = t0, t1 = time.time()) # Checks if a exception is requested

                    # Search_image
                    if(self._step=='Search_image'):
                        # Searchs the image in the article
                        rospy.loginfo('Searching article image')
                        image_url, image_type = self.get_image(article)
                        rospy.loginfo('Image selected: %s (%s)' % (image_url, image_type))
                        self._step = 'Show_info'
                    self.exception_check(t0 = t0, t1 = time.time()) # Checks if a exception is requested

                    # Show_info
                    if(self._step=='Show_info'):
                        # Show info
                        tablet_name_msg, etts_name_msg = self.show_info_handler(parsed_content, article, image_url, image_type)
                        print('Waiting')
                        i=0
                        # Continue when ca has finished
                        while(i<10):
                            self.exception_check(deactivation = [tablet_name_msg, etts_name_msg], t0 = t0, t1 = time.time()) # Checks if a exception is requested
                            i+=1
                            rospy.sleep(1)
                        print('Waited')
                        self._step = 'Cache_update'
                    self.exception_check(t0 = t0, t1 = time.time()) # Checks if a exception is requested

                    # Cache_update
                    if(self._step=='Cache_update'):
                        # Updates the cache with the new article
                        self._cache_manager.cache_update(article['id'])

                        # Update limit variables
                        if(self._limit_method == 'both' or self._limit_method == 'plays'):
                            self._i_plays += 1
                            percentage_plays = int((float(self._i_plays)/float(self._number_plays))*100)
                            rospy.logdebug('percentage plays: %s' % percentage_plays)
                        if(self._limit_method == 'both' or self._limit_method == 'time'):
                            self._time_run += time.time() - t0
                            percentage_time = int((float(self._time_run)/float(self._max_time))*100)
                            rospy.logdebug('percentage time: %s' % percentage_time)

                        self._percentage = percentage_plays if percentage_plays > percentage_time else percentage_time
                        self._percentage = self._percentage if self._percentage<100 else 100
                        rospy.loginfo('percentage: %s' % self._percentage)
                        self._feedback.percentage_completed = self._percentage
                        self._step = 'Get_rss_info'

                    # Exit while
                    self._exec_out = True if self._feedback.percentage_completed>=100 else False # Exit if limits are exceeded (number_plays, max_time)
                    
                #################### Exceptions ####################
                ### Preempted or cancel:
                except ActionlibException:
                    rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)
                    self._exec_out = True
                    if(self._status == self.STOPPED):
                        self._feedback.app_status = 'stop_ok'
                    else:
                        self._feedback.app_status = 'cancel_ok'
                    self._result.skill_result = self._result.FAIL # Preempted
                ### Error
                except ErrorException as e:
                    rospy.logerr(e)
                    self._exec_out = True
                    self._result.skill_result = self._result.ERROR # Error
                ### Pause
                except PauseException:
                    rospy.logwarn('[%s] Paused' % pkg_name)
                    self._feedback.app_status = 'pause_ok'
                    self._exec_out = False
                #=================== Exceptions ===================#

                # Publish feedback at the end of loop
                self._as.publish_feedback(self._feedback)

            #===================== Exec while =====================#
            self._goal_exec = False # Goal execution finished
            print('\n')
        #==================== Skill activa ========================#

        ############# Si la skill no esta activa: ##################
        else:
            rospy.logwarn("STOPPED")
            rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
            self._result.skill_result = self._result.ERROR # Error
        #==========================================================#
        
        #### Envio del resultado y actualizacion del status del goal ###
        # Send result
        if self._result.skill_result == self._result.SUCCESS:
            rospy.logdebug("setting goal to succeeded")
            self._feedback.app_status = 'completed_ok'
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._feedback.app_status = 'completed_fail'
            self._as.publish_feedback(self._feedback)
            self._as.set_preempted(self._result)
        rospy.loginfo("#############################")
        rospy.loginfo("######## Result sent ########")
        rospy.loginfo("#############################")
        
        #==============================================================#

        


if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name, log_level=rospy.DEBUG)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = NewspaperSkill()
        rospy.sleep(1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
