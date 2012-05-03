$(document).ready(function() {
	// settings
	var pointLimit = 30;
	var imagePoints = 1;
	var iframePoints = 6;
	// loop through sections adding up points until the limit is reached
	var i = 0;
	var currentPoints = 0;
	var sectionsHidden = false;
	var sections = document.getElementsByTagName('section');
	for (; i < sections.length; i++) {
		var $section = $(sections[i]);
		currentPoints += $section.find('img').length * imagePoints;
		currentPoints += $section.find('iframe').length * iframePoints;
		if (currentPoints > pointLimit) {
			break;
		}
	}
	// make sure at least one section is left displayed
	if (i == 0) {
		i = 1;
	}
	// hide the remaining sections
	for (; i < sections.length; i++) {
		sectionsHidden = true;
		sections[i].style.display = 'none';
	}
	// setup a 'more' button if sections were hidden
	if (sectionsHidden) {
		$('#content').append($('<div id="more">Show More</div>'));
		$('#more').click(function() {
			// this is run when the 'more' button is clicked
			var sections = document.getElementsByTagName('section');
			for (var i = 0; i < sections.length; i++) {
				if (sections[i].style.display == 'none') {
					sections[i].style.display = 'block';
					if (i == sections.length - 1) {
						$('#more').hide();
					}
					break;
				}
			}
		});
	}
});
