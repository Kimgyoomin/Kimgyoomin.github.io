const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

const canvas = document.getElementById('pointcloud');
if (canvas) {
  const ctx = canvas.getContext('2d');
  let particles = [];
  let width = 0;
  let height = 0;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);

  function initParticles() {
    const count = Math.min(180, Math.max(80, Math.floor(width / 8)));
    particles = Array.from({ length: count }, (_, i) => {
      const band = i % 3;
      return {
        x: Math.random() * width,
        y: Math.random() * height,
        vx: (Math.random() - 0.5) * 0.16,
        vy: (Math.random() - 0.5) * 0.09,
        r: Math.random() * 1.4 + 0.4,
        a: Math.random() * 0.45 + 0.12,
        band
      };
    });
  }

  function resizePointCloud() {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    initParticles();
  }

  function drawPointCloud() {
    ctx.clearRect(0, 0, width, height);
    for (const p of particles) {
      if (!prefersReducedMotion) {
        p.x += p.vx;
        p.y += p.vy;
        if (p.x < -10) p.x = width + 10;
        if (p.x > width + 10) p.x = -10;
        if (p.y < -10) p.y = height + 10;
        if (p.y > height + 10) p.y = -10;
      }

      const color = p.band === 0 ? `rgba(125,255,214,${p.a})`
        : p.band === 1 ? `rgba(149,191,255,${p.a * 0.78})`
        : `rgba(255,255,255,${p.a * 0.5})`;
      ctx.beginPath();
      ctx.fillStyle = color;
      ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2);
      ctx.fill();
    }
    requestAnimationFrame(drawPointCloud);
  }

  window.addEventListener('resize', resizePointCloud, { passive: true });
  resizePointCloud();
  drawPointCloud();
}

const revealObserver = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    if (entry.isIntersecting) {
      entry.target.classList.add('visible');
      revealObserver.unobserve(entry.target);
    }
  });
}, { threshold: 0.14 });

document.querySelectorAll('.reveal').forEach((el) => revealObserver.observe(el));

const languageButton = document.getElementById('langToggle');
let language = 'en';

function setLanguage(nextLanguage) {
  language = nextLanguage;
  document.documentElement.lang = language;
  document.querySelectorAll('[data-en][data-ko]').forEach((el) => {
    el.textContent = el.dataset[language];
  });
  if (languageButton) {
    languageButton.textContent = language === 'en' ? 'KR' : 'EN';
    languageButton.setAttribute('aria-label', language === 'en' ? '한국어로 보기' : 'View in English');
  }
  localStorage.setItem('portfolio-language', language);
}

if (languageButton) {
  languageButton.addEventListener('click', () => {
    setLanguage(language === 'en' ? 'ko' : 'en');
  });
}

const savedLanguage = localStorage.getItem('portfolio-language');
if (savedLanguage === 'ko' || savedLanguage === 'en') setLanguage(savedLanguage);

const navLinks = document.querySelectorAll('.nav a[href^="#"]');
navLinks.forEach((link) => {
  link.addEventListener('click', () => {
    navLinks.forEach((item) => item.removeAttribute('aria-current'));
    link.setAttribute('aria-current', 'page');
  });
});
