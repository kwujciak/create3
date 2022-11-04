class Bump_Sound(Node):
    '''
    This class contains a publisher, a subscriber and an action client. 
    It will take in data from a the hazard detection topic - the bumpers, 
    publish to the audio note sequence topic, and send a goal to play
    an audio note. This class "Bump_Sound" which is a subclass of Node. 
    '''
              
    def __init__(self):
    	'''
    	We initialize the class by calling the Node constructor then naming our node 'bump_sound'
    	'''
    	super().__init__('bump_sound')
    	
    	'''
    	Then we initialize the subscriber, publisher and action client
    	'''
    	print('Hazard detection (subscriber), audio note (publisherr), and audio note sequence (action) are all initialized.')
    	self.subscription = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection',self.listener_callback, qos_profile_sensor_data)
    	self.publisher = self.create_publisher(AudioNoteVector, namespace + '/cmd_audio',10)
    	self._action_client = ActionClient(self, AudioNoteSequence, namespace + '/audio_note_sequence')
    	
    	'''
    	rename Create3 messages so that they can be later manipulated with methods
    	'''
    	self.audio_note_vector = AudioNoteVector()
    	self.an = AudioNotes()
    	
    def listener_callback(self, msg):  
        '''
	As mentioned above, we registered a the listener_callback
	function here (show). Thus, whenever your computer
	recieves a message from the hazard detection sensors (the bumpers) 
	this function will be called. In this function, as you're about
	to see, we will send specific actions. Thus, depending on the info
	we get from the bumpers, that indicates which action will
	be executed. 
	'''

        for detection in msg.detections:
	'''
	We're going to that by using a for loop. When we get data from
	the bumpers, this for loop will parse through it to determine 
	which bumper was hit. Then depending on what the message is,
	the goal will change accordingly.
        '''
        	det = detection.header.frame_id
		# here we're just going to make this part of the message
		# into a variable so we can manipulate it easier.
        	
        	if det!= "base_link":
		# base_link is the message when no bumper hit. So we want
		# to say, when not base_link, or when a bumper is hit,
		# continue. 
    		    print(det)
		# now we're going to use a series of if statement to help us
		# carry out the goal that we want to execute. For example,
		# if det == bump_right, so when the rigt bumper is hit,
		# we want to play audioo note1. 
    		    if det =="bump_right":
    		        self.audio_note_vector.notes = [self.an.audionote1]
    		        print('Publishing specific audio note (right bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
			'''
    		        So, we publish an audio note to the audio note vector 
    		        topic. Now we can send out goal so we'll set up the action client.
    		        '''
    		        self._action_client.note_sequence = self.audio_note_vector
			# Then we can send a goal to play the audio note vector
			# that is currently in the topic
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()
    		    elif det == "bump_left":
			# and now we'll repeat that same process for every hazard detection sensor.
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote2]
    		        print('Publishing specific audio note (left bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()
    		    elif det == "bump_front_left":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote3]
    		        print('Publishing specific audio note (left front bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()				
    		    elif det == "bump_front_right":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote4]
    		        print('Publishing specific audio note (right front bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()
    		    elif det == "bump_front_center":
    		        '''
    		        repeat that process but with different notes for each bumper that is hit
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote5]
    		        print('Publishing specific audio note (front bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()
				
    def send_goal(self):
    	'''
	If you remember from last week, this is our send gaol function, and 
	it looks almost the same as last week, but this time we'll using audio
	notes.
	'''
	# we define the goal
    	goal_msg = AudioNoteSequence.Goal()
	# then wait for the action server to become available.
    	print('Waiting for action server to be available...')
    	self._action_client.wait_for_server()
    	
    	print('Action server available. Sending audio note goal to server.')
	# now it will send the goal. As a reminder, this
	# function is being called in each of these if statements.
    	return self._action_client.send_goal_async(goal_msg) 
