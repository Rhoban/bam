// Fill the {actuator} placeholder in the usage code examples with the motor
// chosen via the ?motor=<name> query parameter (set by the buttons on the
// "Identified actuators" page). When no motor is given, xl330 is used as the
// default. Only whitelisted motor names are ever inserted.
(function () {
  "use strict";

  // name -> human-readable label
  var MOTORS = {
    mx64: "MX-64",
    mx106: "MX-106",
    xl320: "XL-320",
    xl330: "XL-330",
    feetech_sts3215_7_4V: "STS3215 (7.4V)",
  };
  var DEFAULT_MOTOR = "xl330";
  var PLACEHOLDER = "{actuator}";

  function selectedMotor() {
    var value = new URLSearchParams(window.location.search).get("motor");
    if (value && Object.prototype.hasOwnProperty.call(MOTORS, value)) {
      return value;
    }
    return DEFAULT_MOTOR;
  }

  function pageHasPlaceholder() {
    var blocks = document.querySelectorAll("div.highlight pre");
    for (var i = 0; i < blocks.length; i++) {
      if (blocks[i].textContent.indexOf(PLACEHOLDER) !== -1) return true;
    }
    return false;
  }

  function updateCodeBlocks(motor) {
    // The placeholder is a contiguous token inside a string literal, so a plain
    // substring replace on innerHTML is enough (motor is whitelisted).
    document.querySelectorAll("div.highlight pre").forEach(function (pre) {
      if (pre.textContent.indexOf(PLACEHOLDER) === -1) return;
      pre.innerHTML = pre.innerHTML.split(PLACEHOLDER).join(motor);
    });
  }

  // Derive the "_static/" URL prefix from an existing asset link, so the motor
  // image resolves correctly regardless of the current page's depth.
  function staticBase() {
    var link = document.querySelector('link[href*="_static/"]');
    if (link) {
      var href = link.getAttribute("href");
      var idx = href.indexOf("_static/");
      if (idx !== -1) return href.slice(0, idx + "_static/".length);
    }
    return "_static/";
  }

  function showBanner(motor) {
    var article =
      document.querySelector("article") ||
      document.querySelector("main") ||
      document.body;
    var heading = article.querySelector("h1");
    var img = staticBase() + "actuator_" + motor + ".png";

    var banner = document.createElement("div");
    banner.className = "admonition tip motor-banner";
    banner.innerHTML =
      '<img class="motor-banner-img" src="' +
      img +
      '" alt="' +
      MOTORS[motor] +
      '">' +
      '<div class="motor-banner-text">' +
      '<p class="admonition-title">Motor selected</p>' +
      "<p>Examples on this page are shown for " +
      '<code class="docutils literal notranslate"><span class="pre">' +
      motor +
      "</span></code> (" +
      MOTORS[motor] +
      "). " +
      '<a href="actuators.html">Choose a different motor</a>.</p>' +
      "</div>";

    if (heading && heading.parentNode) {
      heading.parentNode.insertBefore(banner, heading.nextSibling);
    } else {
      article.insertBefore(banner, article.firstChild);
    }
  }

  function apply() {
    // Only act on pages that actually carry the {actuator} placeholder.
    if (!pageHasPlaceholder()) return;
    var motor = selectedMotor();
    updateCodeBlocks(motor);
    showBanner(motor);
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", apply);
  } else {
    apply();
  }
})();
