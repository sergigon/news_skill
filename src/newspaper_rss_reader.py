"""
* Feedparser in Python:
https://www.pythonforbeginners.com/feedparser/using-feedparser-in-python
* Python RSS Code (Feedparser + Future):
https://wiki.python.org/moin/RssLibraries
* 
https://www.tutorialspoint.com/python/python_reading_rss_feed.htm
* Documentacion:
https://pythonhosted.org/feedparser/
* HTML to Text converter:
https://www.browserling.com/tools/html-to-text
* Feedly
"""
import feedparser
import html2text # To convert html to text (https://pypi.org/project/html2text/)
import datetime
import requests
from io import BytesIO # https://stackoverflow.com/questions/9772691/feedparser-with-timeout

class RssReader():
	"""
	RssRedaer class
	"""

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
		# Files paths
		self.news_cache = 'news_cache.txt'

		# Parser object
		self.parser = html2text.HTML2Text()
		self.parser.images_to_alt = True # Discard image data, only keep alt text
		self.parser.ignore_images = True # Don't include any formatting for images
		self.parser.ignore_links = True # Ignores links
		self.parser.ignore_tables = True
		self.parser.body_width = 1000 # Number of charcaters per line (long number so no '\n' character appears)

		self.rss_feed = self.feeds['libertad_digital']['portada']
	def rss_reader(self):

		# Do request using requests library and timeout
		try:
			print('Making request')
			resp = requests.get(self.rss_feed, timeout=5.05)
		except requests.ReadTimeout as e:
			rospy.logerr("Timeout when reading RSS %s: %s", self.rss_feed, e)
			return -1
		except requests.exceptions.ConnectionError as e:
			rospy.logerr("Connection error; %s", e)
			return -1
		except requests.exceptions.MissingSchema as e:
			rospy.logerr("MissingSchema; %s", e)
			return -1

		# Put it to memory stream object universal feedparser
		content = BytesIO(resp.content)

		# Parse content
		d = feedparser.parse(content)

		############### Cache date managing ###############
		# Actual date
		date = datetime.datetime.now().date()
		# Reads first line of cache
		first_line = '' # First line of the file
		try:
			file = open(self.news_cache, "r") # Read file
			first_line = file.readline().replace("\n", "") # Gets first line
			file.close() # Close file
		except IOError as e: # File does not exists
			print('ERROR: ' + str(e))
		# If date is not updated or not exists, it erases the file and updates the date
		if(first_line[:5] != '_____' or first_line[5:] != str(date)):
			print('Updating cache date')
			file = open(self.news_cache, "w") # Write file
			file.write('_____' + str(date)) # _____{date}
			file.close() # Close file

		# Gets file lines
		file = open(self.news_cache, "r") # Read file
		file_lines = file.readlines() # Get lines
		file.close() # Close file
		for i in range(len(file_lines)): # Erases the line breaks
			file_lines[i] = file_lines[i].replace("\n", "")
		#####################################################

		try:
			print('__________feed:__________')
			print(d['feed']['title'])
			if(d['bozo']==0):
				print 'xml well-formed'
			else:
				print 'ERROR xml not well-formed'

			print('Number of articles: ' + str(len(d['entries'])))
			print('_________________________\n')

			# Loop for the entries
			i = -1
			for entry in d['entries']:
				i+=1
				found = False

				# Checks if id is in the list
				for id_n in file_lines:
					if(entry['id'] == id_n):
						#print('found')
						found = True
						break

				# If id is not in the list, actual article is selected
				if(not found):
					# Show info
					print('Article ' + str(i) + ':')
					print('__________title:__________')
					print(entry['title'])
					print('__________id:__________')
					print(entry['id'])
					print('__________summary:__________')
					print(entry['summary'])
					print('__________summary_detail[type]:__________')
					print(entry['summary_detail']['type'])
					print('__________summary_detail[value]:__________')
					summary_value = entry['summary_detail']['value']
					# Checks if it is necessary to parse the text
					if(entry['summary_detail']['type'] == 'text/html'):
						print('### Summary parsed ###')
						summary_value = self.parser.handle(summary_value)
						# If text ends with '\n', ti is removed
						while(summary_value[-1:] == '\n'):
							summary_value = summary_value[0:-1]
					print(summary_value)

					if 'content' in entry:
						for i in range(0, len(entry['content'])):
							print('__________content[' + str(i) + '][type]:__________')
							print(entry['content'][i]['type'])
							print('__________content[' + str(i) + '][value]:__________')
							content_value = entry['content'][i]['value']
							if(entry['content'][i]['type'] == 'text/html'):
								print('### Content parsed ###')
								content_value = self.parser.handle(content_value)
								# If text ends with '\n', ti is removed
								while(content_value[-1:] == '\n'):
									content_value = content_value[0:-1]
							print(content_value)

					# Updates cache
					file = open(self.news_cache, "w") # Write file
					file_lines.append(str(entry['id']))
					text = ''
					for line in file_lines:
						text += str(line) + '\n'
					file.write(text) # id
					file.close() # Close file
					break

			# All articles have been found
			if(found):
				print('No more news to show')

		except KeyError as e:
			print('KeyError: ' + str(e))
			return -1

		print '##########################'

		return 0
