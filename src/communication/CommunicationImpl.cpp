#include "CommunicationImpl.hpp"
#include "cloud/asio/asio_http/asio_http.hpp"
#include "cloud/service_controller/service_controller.hpp"
#include "cloud/speech/speech_detection_google/speech_detection_google.hpp"
#include "rapp/objects/audio/audio.hpp"
#include <string>
#include <fstream>
#include <streambuf>

namespace rapp {
namespace robot {

CommunicationImpl::CommunicationImpl(int argc,char **argv) {
	ros::init(argc, argv,"Communication_client");
	n = new ros::NodeHandle();
}

CommunicationImpl::~CommunicationImpl() {
}


/*bool CommunicationImpl::textToSpeech(const std::string & str){	

	client_say = n->serviceClient<elektron_msgs::Say>("rapp_say");
	if (!client_say) {
		ROS_ERROR("Can't connect to service rapp_say!");
		return false;
	}
	elektron_msgs::Say srv;
	bool successful = false;
	
	srv.request.request = str;
	srv.request.language="English";
	if (client_say.call(srv))
	{
		ROS_INFO("Nao said");
		successful = true;
		return successful;
	}
	else
	{
		ROS_ERROR("Failed to call service Say"); 
		successful = false;
	}
	return successful;
}*/
	
bool CommunicationImpl::textToSpeech(const std::string & str, Language language) {
	client_say = n->serviceClient<elektron_msgs::Say>("rapp_say");
	if (!client_say) {
		ROS_ERROR("Can't connect to service rapp_say!");
		return false;
	}
	elektron_msgs::Say srv;
	bool successful = false;
	
	ROS_DEBUG("Final sentence: %s", str.c_str());

	srv.request.request = str;  
	switch(language) {
		case Language::ENGLISH: srv.request.language = "english"; break;
		case Language::GREEK: srv.request.language = "greek"; break;
	}
	

	if (client_say.call(srv)) {
		successful = true;
	} else {
		//Failed to call service
		ROS_ERROR("textToSpeech call failed!");
		successful = false;
	}
	
	return successful;
}

bool CommunicationImpl::playAudio(std::string file_path, double position, double volume, double balance, bool play_in_loop) {
	client_playAudio = n->serviceClient<elektron_msgs::PlayAudio>("rapp_play_audio");
	elektron_msgs::PlayAudio srv;
	bool successful = false;
	
	srv.request.file_path = file_path;
	srv.request.begin_position = position;
	srv.request.volume = volume;
	srv.request.balance_LR = balance;
	srv.request.play_in_loop = play_in_loop;

	if (client_playAudio.call(srv)) {
		if (srv.response.success == true)
			successful = true;
		else {
			ROS_ERROR("playAudio call returned false!");
			successful = false;
		}
	} else {
		//Failed to call service
		ROS_ERROR("playAudio call failed!");
		successful = false;
	}
	
	return successful;
}

std::string CommunicationImpl::wordSpotting(const std::vector<std::string>& dictionary, int time){	

	std::string audio_file = captureAudio(time);
	std::vector<std::string> sentences = {};
	rapp::cloud::platform_info info = {"localhost", "9001", "rapp_token"}; 
	rapp::cloud::service_controller ctrl(info);
    std::shared_ptr<rapp::object::audio> audio;
    std::vector<std::string> gram;
    audio = std::make_shared<rapp::object::microphone_wav>(audio_file);
	assert(audio);

    if (audio) {
    // the callback
    auto callback = [&](std::vector<std::string> words, std::vector<std::string> alternatives)
    {
        for (const auto & str : words)
            std::cout << str << " ";
            std::cout << std::endl;
        return words.at(0);
    };

    // make the call
    ctrl.make_call<rapp::cloud::speech_detection_google>(audio,
                                                          "en",
                                                          "rapp_token",
                                                          callback);
            
        }
        else {
            std::cerr << "missing required arguments -- please see \"--help\"\n";
}
}

std::string CommunicationImpl::captureAudio(int time){
	client_say = n->serviceClient<elektron_msgs::Record>("rapp_record");
	elektron_msgs::Record srv;
	
	srv.request.recordingTime = time;
	
	if (client_say.call(srv))
	{
		//ROS_INFO("Nao recorded audio message");
		ROS_INFO_STREAM("Recorded audio message to the file: " << srv.response.recordedFileDest);
		return srv.response.recordedFileDest;
	}
	else
	{
		ROS_ERROR("Failed to call service Record"); 
	}
	return "";
}


int CommunicationImpl::microphoneEnergy(std::string name){
	client_microphoneEnergy = n->serviceClient<elektron_msgs::MicrophoneEnergy>("rapp_get_microphone_energy");
	elektron_msgs::MicrophoneEnergy srv;
	
	srv.request.microphone = name;
	int energy;
	
	if (client_microphoneEnergy.call(srv))
	{
		ROS_INFO_STREAM("Detected microphone energy: " << srv.response.energy);
		energy = srv.response.energy;
	}
	else
	{
		ROS_ERROR("Failed to call service MicrophoneEnergy"); 
		energy = 0;
	}
	return energy;
}


std::string CommunicationImpl::captureAudio (std::string file_path, float waiting_time/*in sec*/, int microphone_energy/*2700*/){
	client_recordWithSoundDetection = n->serviceClient<elektron_msgs::RecordWithSoundDetection>("rapp_record_with_sound_detection");
	elektron_msgs::RecordWithSoundDetection srv;
	
	srv.request.file_dest = file_path;
	srv.request.waiting_time = waiting_time;
	srv.request.microphone_energy = microphone_energy;
	
	if (client_recordWithSoundDetection.call(srv))
	{
		ROS_INFO("Nao recorded audio message");
		return srv.response.output_file_path;
	}
	else
	{
		ROS_ERROR("Failed to call service Record with sound detection"); 
	}
	return "";
}

	
void CommunicationImpl::voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector )
{
	client_voiceRecord = n->serviceClient<elektron_msgs::VoiceRecord>("rapp_voice_record");
	elektron_msgs::VoiceRecord srv;
	
	srv.request.startRecording = startRecording; //if true, the recording will start if false, the recording will be stoped
	
	if (client_voiceRecord.call(srv))
	{
		// for buffer usage
		ROS_INFO_STREAM("Recorded audio message to the buffer. Buffer size: " << srv.response.buffer_.size()); //should be 8192 (microphone buffer size)
		
		audio_buffer_vector.push_back( srv.response.buffer_ ); //adds buffer to the vector of vectors
		return;
	}
	else
	{
		ROS_ERROR("Failed to call service Voice Record"); 
	}
	return;
}

} // namespace robot
} // namespace rapp
