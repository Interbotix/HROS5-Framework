<?php

  require './config.php';
  require './tmhOAuth.php';
  require './tmhUtilities.php';
$token = $_COOKIE['Temp_Token'];
  $secret = $_COOKIE['Temp_Secret'];
  $tmhOAuth = new tmhOAuth(array(
   'consumer_key'    => API_KEY,
    'consumer_secret' => API_SEC,
    'user_token'      => "",
    'user_secret'     => "",	
    'curl_ssl_verifypeer'   => false
  ));


if ($handle = opendir('uploads')) {

    /* This is the correct way to loop over the directory. */
    while (false !== ($entry = readdir($handle))) {
	
	if ($entry != "." && $entry != "..") {
            $image = "uploads/".$entry;
            $dest = "done/".$entry;

       	$code = $tmhOAuth->request('POST', 'https://api.twitter.com/1.1/statuses/update_with_media.json',
  		array(
    		'media[]'  => "@{$image}",
   			//'status'   => "Don't slip up yo" // Don't give up..
  		),
  		true, // use auth
  		true  // multipart
		);
 
		if ($code == 200) {
		  tmhUtilities::pr(json_decode($tmhOAuth->response['response']));
		  	copy($image, $dest);
			unlink($image);
		} else {
		  tmhUtilities::pr($tmhOAuth->response['response']);
		}
		
    }

       

      /* */
    }



    closedir($handle);
}

 


  ?>