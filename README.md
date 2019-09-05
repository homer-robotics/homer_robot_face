# homer_robot_face

![Robot face](./images/robot_face.png)

This package contains a robot face as described in the paper "Enhancing Human-Robot Interaction by a Robot Face with Facial Expressions and Synchronized Lip Movements".


## Instruction

Launch the face node like:

```
roslaunch homer_robot_face robot_face.launch
```

For speech synthesis you can use the `homer_tts` package which contains a collection of
different text to speech synthesizers.


## Topics

* `/recognized_speech` Shows recognized speech in the bottom of the face to support humans in interacting with the robot.
* `/robot_face/ImageDisplay` An Image can be displayed optionally (this topic awaits a image as message).
* `/robot_face/ImageFileDisplay` Alternatively a filename can be specified.
* `/robot_face/expected_input` An additional text on the bottom of the robot face will be shown. Here you can display instructions depending on the state.
* `/robot_face/talking_finished` When coupled with a TTS system this topic will be sent once the robot finished speaking.
* `/robot_face/text_out` When coupled with a TTS system this topic will be used for synthesizing speech.


If you use this package consider citing:

```
@article{seib2013enhancing,
  title={Enhancing human-robot interaction by a robot face with facial expressions and synchronized lip movements},
  author={Seib, Victor and Giesen, Julian and Gr{\"u}ntjens, Dominik and Paulus, Dietrich},
  year={2013},
  publisher={V{\'a}clav Skala-UNION Agency}
}
```

```
@inproceedings{seib2015team,
  title={Team homer@ unikoblenz—approaches and contributions to the robocup@ home competition},
  author={Seib, Viktor and Manthe, Stephan and Memmesheimer, Raphael and Polster, Florian and Paulus, Dietrich},
  booktitle={Robot Soccer World Cup},
  pages={83--94},
  year={2015},
  organization={Springer}
}
```
