use proc_macro::TokenStream;
use quote::quote;
use syn::{DeriveInput, parse_macro_input};

extern crate proc_macro;

#[proc_macro_derive(CommandSerde)]
pub fn robot_serde_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let fields = if let syn::Data::Struct(s) = &input.data {
        &s.fields
    } else {
        panic!("CommandSerde can only be derived for structs");
    };

    let to_string_impl = fields.iter().map(|f| {
        let name = &f.ident;
        quote! {
            CommandSerde::to_string(&self.#name)
        }
    });

    let from_str_init = fields.iter().map(|f| {
        let name = &f.ident; // 字段名
        let ty = &f.ty; // 字段类型
        quote! {
            let #name = <#ty>::from_str(&parts[index..index+<#ty>::num_args()-1].join(","))?;
            index += <#ty>::num_args();

        }
    });

    let try_default_impl = fields.iter().map(|f| {
        let name = &f.ident;
        let ty = &f.ty;
        quote! {
            #name: <#ty>::try_default()
        }
    });

    // 4) 计算 num_args()：把所有字段的 num_args() 累加
    let num_args_impl = fields.iter().map(|f| {
        let ty = &f.ty;
        quote! {
            <#ty>::num_args()
        }
    });

    let field_names = fields.iter().map(|f| {
        let name = &f.ident;
        quote! { #name }
    });

    // 6) 拼装最终要展开的实现
    let expanded = quote! {
        impl CommandSerde for #name {
            fn to_string(&self) -> String {
                vec![#(#to_string_impl),*].join(",")
            }

            fn from_str(s: &str) -> HansResult<Self> {
                let parts: Vec<&str> = s.split(',').collect();
                let mut index = 0;

                #(#from_str_init)*

                Ok(Self {
                    #(#field_names),*
                })
            }

            fn try_default() -> Self {
                Self {
                    #(#try_default_impl),*
                }
            }

            fn num_args() -> usize {
                vec![#(#num_args_impl),*].iter().sum()
            }
        }
    };

    TokenStream::from(expanded)
}
