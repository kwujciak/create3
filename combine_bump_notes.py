class Bump_Sound(Node):
    '''
    This class contains a publisher, a subscriber and an action client. It will take in data from
    a the hazard detection topic, publish to the audio note sequence topic, and send a goal to play
    an audio note. We are defining a class "Bump_Sound" which is a subclass of Node. 
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
        Whenever the computer recieves a message from the hazard detection sensors this function will be called
        and any actions in this function will be executed.

        First we parse the message to determine which bumper was hit. 
        Then depending on what the message change we change the goal accordingly.
        '''
        for detection in msg.detections:
        	det = detection.header.frame_id
        	
        	if det!= "base_link":
    		    print(det)
    		    if det =="bump_right":
    		        '''
    		        in order to play an audio note we need to publish an audio note to the audio note vector 
    		        topic. Then we can send a goal to play the audio note vector that is currently in the topic
    		        '''
    		        self.audio_note_vector.notes = [self.an.audionote1]
    		        print('Publishing specific audio note (right bumper hit) to audio note vector.')
    		        self.publisher.publish(self.audio_note_vector)
    		        self._action_client.note_sequence = self.audio_note_vector
    		        print('Sending goal to play audio note vector.')
    		        self.send_goal()
    		    elif det == "bump_left":
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
	this function defines the goal, waits for the action server to be available
	and then sends the goal to the action server. It gets called in the timer_callback()
	function whenever we need to send a goal to the robot
	'''
    	goal_msg = AudioNoteSequence.Goal()
    	print('Waiting for action server to be available...')
    	self._action_client.wait_for_server()
    	
    	print('Action server available. Sending audio note goal to server.')	
    	return self._action_client.send_goal_async(goal_msg) 
