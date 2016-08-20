:set tabstop=4
:set shiftwidth=4
:set expandtab
:set autoindent
:set smartindent
:set cindent

if has("autocmd")
  au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
      \| exe "normal! g'\"" | endif
endif
