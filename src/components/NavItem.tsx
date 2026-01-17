'use client';

interface NavItemProps {
  label: string;
  target: string;
  active: boolean;
}

export default function NavItem({ label, target, active }: NavItemProps) {
  const handleClick = () => {
    document.getElementById(target)?.scrollIntoView({ behavior: 'smooth' });
  };

  return (
    <button 
      onClick={handleClick}
      className={`w-full text-left py-2 px-6 text-xs font-medium transition-all duration-300 border-l-2 cursor-pointer ${
        active 
          ? 'border-zinc-900 text-zinc-900 bg-zinc-50' 
          : 'border-transparent text-zinc-400 hover:text-zinc-600 hover:bg-zinc-50/50'
      }`}
    >
      {label}
    </button>
  );
}
