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

    let from_str_impl = fields.iter().enumerate().map(|(i, f)| {
        let name = &f.ident;
        let ty = &f.ty;
        quote! {
            #name: <#ty>::from_str(&parts[#i])?
        }
    });

    let try_default_impl = fields.iter().map(|f| {
        let name = &f.ident;
        let ty = &f.ty;
        quote! {
            #name: <#ty>::try_default()
        }
    });

    let expanded = quote! {
        impl CommandSerde for #name {
            fn to_string(&self) -> String {
                vec![#(#to_string_impl),*].join(",")
            }

            fn from_str(s: &str) -> HansResult<Self> {
                let parts: Vec<&str> = s.split(',').collect();
                Ok(#name {
                    #(#from_str_impl),*
                })
            }

            fn try_default() -> Self {
                #name {
                    #(#try_default_impl),*
                }
            }
        }
    };

    TokenStream::from(expanded)
}
