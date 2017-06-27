# homer_tts

This package holds multiple ways to generate speech by text:

* Mary TTS http://mary.dfki.de/
* flite: A small fast runtime synthesis engine
* pico2wave: This uses an android speech engine
* More to come

We have tried multiple approaches that have different favors and bottlenecks. 

## Mary TTS
First Mary TTS generates good sounding speech and has an easy interface for installing
new voices but consumes a lot of memory. Therefore it is not suitable for the use
on single board computers as of now.

### Installation

Head over to http://mary.dfki.de/download/index.html and grab an installer. 
The mary tts node assumes that you have started the mary tts server beforehand on
the standard port.

# Flite 

This is a small and fast version of festival. However, with the voices and params
we tried speech generation is still to slow for single board computers.

### Installation

```
sudo apt-get install flite
```

# Pico2Wave

This tool speech synthesis engine is really fast for usage on single board computers. 
There are not many parameters existing but some languages as english and german.

### Installation

```
sudo apt-get install libttspico-utils
```

Mary TTS bibtex:


```
@article{schroder2003german,
  title={The German text-to-speech synthesis system MARY: A tool for research, development and teaching},
  author={Schr{\"o}der, Marc and Trouvain, J{\"u}rgen},
  journal={International Journal of Speech Technology},
  volume={6},
  number={4},
  pages={365--377},
  year={2003},
  publisher={Springer}
}
```

Flite bibtex:

```
@inproceedings{black2001flite,
  title={Flite: a small fast run-time synthesis engine},
  author={Black, Alan W and Lenzo, Kevin A},
  booktitle={4th ISCA Tutorial and Research Workshop (ITRW) on Speech Synthesis},
  year={2001}
}
```
