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

import datetime
import rospy

class CacheManager():
	"""
	Cache manager class.
	"""

	def __init__(self, datapath):
		"""
		Init method.

		@param datapath: Data path from where to work with data. Usually, the package data path.
		"""

		# Gets paths
		self._data_path = datapath # Data path
		# Get file paths
		self._news_cache = self._data_path + 'news_cache.txt' # Cache files

	def cache_refresh(self):
		"""
		Refresh the cache.

		If cache does not exist, it is created.
		If date is not updated, it erases the content and updates the date.
		"""

		rospy.logdebug('Refreshing cache')

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

		@param url: url of the new article.
		'''

		file_lines = self.cache_get_id()

		file = open(self._news_cache, "w") # Write file
		file_lines.append(str(url))
		text = ''
		for line in file_lines:
			text += str(line) + '\n'
		file.write(text) # id
		file.close() # Close file

	def cache_get_id(self):
		"""
		Gets ids from the cache and export them to the variable self._file_lines.

		@return file_lines: Content of the cache (ids).
		"""

		file = open(self._news_cache, "r") # Read file
		file_lines = []
		file_lines = file.readlines() # Get lines
		file.close() # Close file
		for i in range(len(file_lines)): # Erases the line breaks
			file_lines[i] = file_lines[i].replace("\n", "")

		return file_lines