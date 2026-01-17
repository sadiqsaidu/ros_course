import { BookOpen, Clock, User, ArrowRight, CheckCircle, Circle, Lock } from 'lucide-react';
import Link from 'next/link';

interface LectureCardProps {
  number: number;
  title: string;
  description: string;
  status: 'completed' | 'current' | 'upcoming' | 'locked';
  topics: string[];
  href?: string;
}

function LectureCard({ number, title, description, status, topics, href }: LectureCardProps) {
  const isAccessible = status === 'completed' || status === 'current';
  
  const statusConfig = {
    completed: {
      badge: 'Completed',
      badgeClass: 'bg-green-100 text-green-700',
      icon: <CheckCircle size={20} className="text-green-500" />,
    },
    current: {
      badge: 'Current',
      badgeClass: 'bg-blue-100 text-blue-700',
      icon: <Circle size={20} className="text-blue-500 fill-blue-500" />,
    },
    upcoming: {
      badge: 'Coming Soon',
      badgeClass: 'bg-zinc-100 text-zinc-500',
      icon: <Circle size={20} className="text-zinc-300" />,
    },
    locked: {
      badge: 'Locked',
      badgeClass: 'bg-zinc-100 text-zinc-400',
      icon: <Lock size={16} className="text-zinc-400" />,
    },
  };

  const config = statusConfig[status];

  const content = (
    <>
      <div className="flex items-start justify-between mb-4">
        <div className="flex items-center space-x-3">
          <div className={`w-12 h-12 rounded-xl flex items-center justify-center font-bold text-lg ${
            isAccessible ? 'bg-zinc-900 text-white' : 'bg-zinc-100 text-zinc-400'
          }`}>
            {number}
          </div>
          <div>
            <span className={`text-xs font-medium px-2 py-1 rounded-full ${config.badgeClass}`}>
              {config.badge}
            </span>
          </div>
        </div>
        {config.icon}
      </div>
      
      <h3 className={`text-xl font-bold mb-2 ${isAccessible ? 'text-zinc-900' : 'text-zinc-400'}`}>
        {title}
      </h3>
      <p className={`text-sm mb-4 ${isAccessible ? 'text-zinc-600' : 'text-zinc-400'}`}>
        {description}
      </p>
      
      <div className="flex flex-wrap gap-2">
        {topics.map((topic, i) => (
          <span 
            key={i} 
            className={`text-xs px-2 py-1 rounded ${
              isAccessible ? 'bg-zinc-100 text-zinc-600' : 'bg-zinc-50 text-zinc-400'
            }`}
          >
            {topic}
          </span>
        ))}
      </div>

      {isAccessible && (
        <div className="mt-4 pt-4 border-t border-zinc-100 flex items-center text-sm font-medium text-zinc-900">
          View Lecture <ArrowRight size={14} className="ml-1" />
        </div>
      )}
    </>
  );

  if (isAccessible && href) {
    return (
      <Link 
        href={href}
        className={`block p-6 bg-white border rounded-2xl transition-all duration-300 border-zinc-200 hover:border-zinc-400 hover:shadow-lg cursor-pointer`}
      >
        {content}
      </Link>
    );
  }

  return (
    <div className={`block p-6 bg-white border rounded-2xl transition-all duration-300 border-zinc-100 opacity-60`}>
      {content}
    </div>
  );
}

export default function HomePage() {
  const lectures: LectureCardProps[] = [
    {
      number: 1,
      title: 'ROS 2 Foundations',
      description: 'Introduction to ROS 2, understanding nodes, and creating your first program.',
      status: 'current',
      topics: ['What is ROS 2', 'Nodes', 'Installation', 'First Node'],
      href: '/lectures/1',
    },
    {
      number: 2,
      title: 'Topics & Publishers',
      description: 'Learn how nodes communicate through topics using publish-subscribe pattern.',
      status: 'upcoming',
      topics: ['Topics', 'Publishers', 'Subscribers', 'Messages'],
    },
    {
      number: 3,
      title: 'Services & Actions',
      description: 'Request-response communication and long-running task management.',
      status: 'locked',
      topics: ['Services', 'Actions', 'Clients', 'Servers'],
    },
    {
      number: 4,
      title: 'Building the Robot',
      description: 'Putting it all together to build our autonomous delivery robot.',
      status: 'locked',
      topics: ['Integration', 'Launch Files', 'Packages', 'Testing'],
    },
  ];

  return (
    <div className="min-h-screen bg-zinc-50">
      {/* Header */}
      <header className="bg-white border-b border-zinc-200">
        <div className="max-w-6xl mx-auto px-6 py-6">
          <div className="flex items-center space-x-3">
            <div className="w-12 h-12 bg-zinc-900 text-white flex items-center justify-center rounded-xl shadow-lg">
              <span className="font-bold font-mono text-xl">R2</span>
            </div>
            <div>
              <h1 className="font-bold text-xl tracking-tight text-zinc-900">ROS 2 Fundamentals</h1>
              <p className="text-sm text-zinc-500">Building an Autonomous Delivery Robot</p>
            </div>
          </div>
        </div>
      </header>

      {/* Hero Section */}
      <section className="bg-white border-b border-zinc-200">
        <div className="max-w-6xl mx-auto px-6 py-16 md:py-24">
          <div className="max-w-3xl">
            <h2 className="text-4xl md:text-5xl font-extrabold text-zinc-900 mb-6 leading-tight">
              Learn to Build <br/>
              <span className="text-transparent bg-clip-text bg-gradient-to-r from-blue-600 to-purple-600">
                Intelligent Robots
              </span>
            </h2>
            <p className="text-xl text-zinc-600 mb-8 leading-relaxed">
              A project-based course on ROS 2 (Robot Operating System). By the end, you&apos;ll understand 
              how real robots are structured and how software makes them intelligent.
            </p>
            
            <div className="flex flex-wrap gap-6 mb-8">
              <div className="flex items-center space-x-2 text-zinc-600">
                <User size={18} />
                <span className="text-sm font-medium">Abubakar-Sadiq Saidu</span>
              </div>
              <div className="flex items-center space-x-2 text-zinc-600">
                <Clock size={18} />
                <span className="text-sm font-medium">4 Weeks</span>
              </div>
              <div className="flex items-center space-x-2 text-zinc-600">
                <BookOpen size={18} />
                <span className="text-sm font-medium">Project-Based</span>
              </div>
            </div>

            <Link 
              href="/lectures/1" 
              className="inline-flex items-center px-6 py-3 bg-zinc-900 text-white rounded-full font-medium hover:bg-zinc-800 transition-colors"
            >
              Start Learning <ArrowRight size={16} className="ml-2" />
            </Link>
          </div>
        </div>
      </section>

      {/* Course Overview */}
      <section className="py-16 md:py-24">
        <div className="max-w-6xl mx-auto px-6">
          <div className="mb-12">
            <h2 className="text-3xl font-bold text-zinc-900 mb-4">Course Lectures</h2>
            <p className="text-zinc-600">Progressive lessons building towards your autonomous robot.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            {lectures.map((lecture) => (
              <LectureCard key={lecture.number} {...lecture} />
            ))}
          </div>
        </div>
      </section>

      {/* What You'll Learn */}
      <section className="py-16 md:py-24 bg-white border-t border-zinc-200">
        <div className="max-w-6xl mx-auto px-6">
          <div className="mb-12">
            <h2 className="text-3xl font-bold text-zinc-900 mb-4">What You&apos;ll Learn</h2>
            <p className="text-zinc-600">Skills and concepts you&apos;ll master in this course.</p>
          </div>

          <div className="grid md:grid-cols-3 gap-8">
            <div className="p-6 bg-zinc-50 rounded-2xl">
              <div className="w-12 h-12 bg-blue-100 rounded-xl flex items-center justify-center mb-4 text-2xl">
                🧠
              </div>
              <h3 className="font-bold text-zinc-900 mb-2">Systems Thinking</h3>
              <p className="text-sm text-zinc-600">
                Understand how complex robot software is structured as cooperating, independent modules.
              </p>
            </div>
            <div className="p-6 bg-zinc-50 rounded-2xl">
              <div className="w-12 h-12 bg-green-100 rounded-xl flex items-center justify-center mb-4 text-2xl">
                🔧
              </div>
              <h3 className="font-bold text-zinc-900 mb-2">ROS 2 Mastery</h3>
              <p className="text-sm text-zinc-600">
                Master nodes, topics, services, and actions — the core building blocks of ROS 2.
              </p>
            </div>
            <div className="p-6 bg-zinc-50 rounded-2xl">
              <div className="w-12 h-12 bg-purple-100 rounded-xl flex items-center justify-center mb-4 text-2xl">
                🤖
              </div>
              <h3 className="font-bold text-zinc-900 mb-2">Real Robot Architecture</h3>
              <p className="text-sm text-zinc-600">
                Apply patterns used in industrial robots, self-driving cars, and drones.
              </p>
            </div>
          </div>
        </div>
      </section>

      {/* Prerequisites */}
      <section className="py-16 md:py-24 border-t border-zinc-200">
        <div className="max-w-6xl mx-auto px-6">
          <div className="mb-12">
            <h2 className="text-3xl font-bold text-zinc-900 mb-4">Prerequisites</h2>
            <p className="text-zinc-600">What you need before starting this course.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-6">
            <div className="p-6 bg-white border border-zinc-200 rounded-2xl">
              <h3 className="font-bold text-zinc-900 mb-4">Required</h3>
              <ul className="space-y-3 text-zinc-600">
                <li className="flex items-center space-x-2">
                  <CheckCircle size={16} className="text-green-500" />
                  <span>Ubuntu 22.04 (or VM/WSL2)</span>
                </li>
                <li className="flex items-center space-x-2">
                  <CheckCircle size={16} className="text-green-500" />
                  <span>Basic Python syntax</span>
                </li>
                <li className="flex items-center space-x-2">
                  <CheckCircle size={16} className="text-green-500" />
                  <span>Basic terminal/command line usage</span>
                </li>
              </ul>
            </div>
            <div className="p-6 bg-white border border-zinc-200 rounded-2xl">
              <h3 className="font-bold text-zinc-900 mb-4">Not Required</h3>
              <ul className="space-y-3 text-zinc-600">
                <li className="flex items-center space-x-2">
                  <Circle size={16} className="text-zinc-300" />
                  <span>Prior ROS or ROS 2 experience</span>
                </li>
                <li className="flex items-center space-x-2">
                  <Circle size={16} className="text-zinc-300" />
                  <span>Robotics hardware</span>
                </li>
                <li className="flex items-center space-x-2">
                  <Circle size={16} className="text-zinc-300" />
                  <span>Advanced programming skills</span>
                </li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="bg-zinc-900 text-white py-12">
        <div className="max-w-6xl mx-auto px-6">
          <div className="flex flex-col md:flex-row justify-between items-center">
            <div className="flex items-center space-x-3 mb-4 md:mb-0">
              <div className="w-10 h-10 bg-white text-zinc-900 flex items-center justify-center rounded-lg">
                <span className="font-bold font-mono">R2</span>
              </div>
              <div>
                <div className="font-bold">ROS 2 Fundamentals</div>
                <div className="text-xs text-zinc-400">RAIN-INN ABU</div>
              </div>
            </div>
            <div className="text-sm text-zinc-400">
              © 2026 Abubakar-Sadiq Saidu. All rights reserved.
            </div>
          </div>
        </div>
      </footer>
    </div>
  );
}
