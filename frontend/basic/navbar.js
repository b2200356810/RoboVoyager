let activePage = "";
let isMenuActive = false;
let isDarkTheme = false;
let isDesktop = false;

document.addEventListener("DOMContentLoaded", function () {
    // activePage = window.location.pathname; // Uncomment this line if needed
    isDarkTheme = localStorage.getItem("theme") === "dark";
    updateTheme();
    isDesktop = window.innerWidth >= 700;
    window.addEventListener("resize", handleResize);
});

function handleResize() {
    isDesktop = window.innerWidth >= 700;
    if (window.innerWidth > 700) {
        document.body.classList.remove("menu-active");
        isMenuActive = false;
        // Add logic here to handle other changes on resize if needed
    }
}

function toggleMenu() {
    isMenuActive = !isMenuActive;
    if (isMenuActive) {
        document.querySelector("nav ul").classList.add("active");
    } else {
        document.querySelector("nav ul").classList.remove("active");
    }
}

function toggleTheme() {
    isDarkTheme = !isDarkTheme;
    localStorage.setItem("theme", isDarkTheme ? "dark" : "light");
    updateTheme();
}

function updateTheme() {
    const favicon = document.querySelector("[rel=icon]");
    if (isDarkTheme) {
        favicon.setAttribute("href", "images/roboDark.svg");
        document.body.classList.add("dark-theme");
    } else {
        favicon.setAttribute("href", "images/roboLight.svg");
        document.body.classList.remove("dark-theme");
    }
}

function handleNavLinkClick(event) {
    const links = document.querySelectorAll("nav ul li a");
    links.forEach((link) => link.classList.remove("active"));
    clickedElement.classList.add("active");
    if (window.innerWidth < 700) toggleMenu();

    // event.preventDefault();
    // const targetHref = event.target.getAttribute("href");

    // const menu = document.querySelector("nav ul");
    // menu.addEventListener("transitionend", function transitionEndHandler() {
    //     menu.removeEventListener("transitionend", transitionEndHandler);
    //     window.location.href = targetHref;
    // });
}
