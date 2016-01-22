TangoLogger
===========

An Arduino based electric vehicle logger that gathers data from either a Sevcon Gen4 or Kelly KLS8080I motor controller, GPS, a hall sensor, and has the ability to upload data via wifi.

##Libraries Used
Some libraries that are either written by me or have permissive licenses are in this git repo.  This code uses software that uses the GPL
and thus you should get it yourself:

* [PString version 3: http://arduiniana.org/libraries/pstring/](http://arduiniana.org/libraries/pstring/)
* [TinyGPS version 13: http://arduiniana.org/libraries/tinygps/](http://arduiniana.org/libraries/tinygps/)
* sdfatlib version 20131225.  Unfortunately, that specific version doesn't seem to be available anymore, and the new version of SdFat that
  has been moved to github is incompatible from an API point of view.  I imagine it's not too hard to port the code to use the new SdFat
  library, but I haven't tried.  If you can find an old version 20131225, that's what I've used in this codebase.

##License
Copyright 2011 Alex Tang. <altitude@funkware.com>

This code is released under the GPL v3.  However, if you want to use it
in a commercial or non-GPL way, I'm likely going to say "sure!" but I'd
like to know about it first. So if you're intersted in using any/all
of this code in a way that's not condusive to the GPL, drop me an
email.
