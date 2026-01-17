'use client';

import { useState, useEffect } from 'react';
import { Layout, LucideIcon } from 'lucide-react';
import NavItem from './NavItem';

export interface SlideInfo {
  id: string;
  title: string;
  icon: LucideIcon;
}

interface LectureSidebarProps {
  slides: SlideInfo[];
  lectureNumber: number;
  lectureTitle: string;
}

export default function LectureSidebar({ slides, lectureNumber, lectureTitle }: LectureSidebarProps) {
  const [activeSection, setActiveSection] = useState('slide-1');
  const [isMenuOpen, setIsMenuOpen] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            setActiveSection(entry.target.id);
          }
        });
      },
      { threshold: 0.2, rootMargin: "-20% 0px -50% 0px" }
    );

    document.querySelectorAll('section').forEach((section) => {
      observer.observe(section);
    });

    return () => observer.disconnect();
  }, []);

  return (
    <>
      {/* Mobile Menu Toggle */}
      <button 
        onClick={() => setIsMenuOpen(!isMenuOpen)}
        className="fixed top-4 right-4 z-50 p-2 bg-white/80 backdrop-blur border border-zinc-200 text-zinc-900 rounded-full shadow-lg md:hidden hover:bg-zinc-100 cursor-pointer"
      >
        <Layout size={20} />
      </button>

      {/* Sidebar Navigation */}
      <nav className={`fixed inset-y-0 left-0 w-72 bg-white border-r border-zinc-200 transform transition-transform duration-300 ease-in-out z-40 ${isMenuOpen ? 'translate-x-0' : '-translate-x-full'} md:translate-x-0 flex flex-col`}>
        <div className="p-8 pb-6 border-b border-zinc-100">
          <a href="/" className="flex items-center space-x-3 text-zinc-900 mb-2 hover:opacity-80 transition-opacity">
            <div className="w-10 h-10 bg-zinc-900 text-white flex items-center justify-center rounded-lg shadow-lg">
              <span className="font-bold font-mono text-lg">R2</span>
            </div>
            <div>
              <div className="font-bold tracking-tight leading-none">ROS 2</div>
              <div className="text-xs text-zinc-500 font-medium tracking-wide uppercase">Fundamentals</div>
            </div>
          </a>
        </div>
        <div className="flex-1 overflow-y-auto py-4">
          <div className="px-6 mb-2 text-xs font-bold text-zinc-400 uppercase tracking-widest">
            Lecture {lectureNumber}: {lectureTitle}
          </div>
          {slides.map((slide) => (
            <NavItem 
              key={slide.id} 
              target={slide.id} 
              label={`${slide.id.replace('slide-', '')}. ${slide.title}`} 
              active={activeSection === slide.id} 
            />
          ))}
        </div>
        <div className="p-6 border-t border-zinc-100">
          <a href="/" className="text-xs text-zinc-400 hover:text-zinc-600 transition-colors">
            ← Back to Course Overview
          </a>
        </div>
        <div className="px-6 pb-6 text-xs text-zinc-400">
          © 2026 Robotics Course
        </div>
      </nav>
    </>
  );
}
